#!/usr/bin/env python3

# Copyright 2026 TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Standalone StreamPETR camera mask designer.

This helper is intentionally separate from the StreamPETR component. It discovers
available sensor_msgs/Image and sensor_msgs/CompressedImage topics, subscribes to
all of them, pairs each stream with a CameraInfo topic when possible, undistorts
the latest frame with the same OpenCV map setup used by StreamPETR, and serves a
small browser UI for drawing/exporting mask polygons.
"""

from __future__ import annotations

import argparse
import base64
import errno
import hashlib
import json
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional
from urllib.parse import parse_qs, unquote, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>StreamPETR Live Camera Mask Designer</title>
  <style>
    :root {
      --bg: #f4f5f7;
      --panel: #fff;
      --ink: #1f2933;
      --muted: #667085;
      --line: #d0d5dd;
      --accent: #006c67;
      --accent-strong: #004f4b;
      --danger: #b42318;
      --canvas-bg: #101828;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
      background: var(--bg);
    }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 410px;
      gap: 16px;
      min-height: 100vh;
      padding: 16px;
    }
    .toolbar, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .toolbar {
      display: grid;
      grid-template-columns: minmax(260px, 1fr) auto auto auto auto;
      gap: 10px;
      align-items: end;
      margin-bottom: 12px;
      padding: 10px;
    }
    .panel { padding: 12px; margin-bottom: 12px; }
    .canvas-shell {
      display: grid;
      place-items: center;
      min-height: calc(100vh - 112px);
      padding: 12px;
      background: var(--canvas-bg);
      border-radius: 8px;
      overflow: auto;
    }
    canvas {
      max-width: 100%;
      max-height: calc(100vh - 136px);
      background: #182230;
      cursor: crosshair;
    }
    label {
      display: grid;
      gap: 5px;
      color: var(--muted);
      font-size: 12px;
      font-weight: 650;
    }
    select, input[type="number"], input[type="text"], textarea {
      width: 100%;
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
      color: var(--ink);
      background: #fff;
      font: inherit;
      font-size: 13px;
    }
    textarea {
      min-height: 110px;
      resize: vertical;
      font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
      line-height: 1.45;
    }
    button {
      height: 36px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 0 12px;
      color: var(--ink);
      background: #fff;
      font: inherit;
      font-size: 13px;
      font-weight: 700;
      cursor: pointer;
      white-space: nowrap;
    }
    button.primary { border-color: var(--accent); color: #fff; background: var(--accent); }
    button.primary:hover { background: var(--accent-strong); }
    button.danger { border-color: #fecdca; color: var(--danger); background: #fff5f5; }
    .row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      align-items: end;
      margin-bottom: 10px;
    }
    .button-row { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 10px; }
    .check {
      display: flex;
      align-items: center;
      gap: 8px;
      min-height: 36px;
      color: var(--ink);
      font-size: 13px;
      font-weight: 700;
    }
    .stat {
      display: grid;
      grid-template-columns: 116px minmax(0, 1fr);
      gap: 8px;
      padding: 5px 0;
      border-bottom: 1px solid #eef2f6;
      font-size: 13px;
    }
    .stat:last-child { border-bottom: 0; }
    .stat span:first-child { color: var(--muted); font-weight: 700; }
    .side h1, .side h2 { margin: 0 0 10px; font-size: 15px; line-height: 1.3; }
    .side h1 { font-size: 18px; }
    .status {
      color: var(--muted);
      font-size: 13px;
      min-height: 18px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    @media (max-width: 1080px) {
      main { grid-template-columns: 1fr; }
      .toolbar { grid-template-columns: 1fr 1fr; }
      .canvas-shell { min-height: 54vh; }
      canvas { max-height: 64vh; }
    }
  </style>
</head>
<body>
  <main>
    <section class="workspace">
      <div class="toolbar">
        <label>
          Camera stream
          <select id="streamSelect"></select>
        </label>
        <button id="refreshButton" type="button">Refresh</button>
        <label class="check"><input id="liveToggle" type="checkbox" checked> Live</label>
        <button id="freezeButton" type="button">Freeze</button>
        <span id="status" class="status"></span>
      </div>
      <div class="canvas-shell">
        <canvas id="maskCanvas" width="1280" height="720"></canvas>
      </div>
    </section>

    <aside class="side">
      <div class="panel">
        <h1>StreamPETR Camera Mask</h1>
        <div class="row">
          <label>
            Camera ID
            <input id="cameraId" type="number" min="0" max="99" value="0">
          </label>
          <label class="check"><input id="normalized" type="checkbox" checked> Normalized</label>
        </div>
        <div class="row">
          <label>
            Fill B
            <input id="fillB" type="number" min="0" max="255" value="0">
          </label>
          <label>
            Fill G
            <input id="fillG" type="number" min="0" max="255" value="0">
          </label>
        </div>
        <div class="row">
          <label>
            Fill R
            <input id="fillR" type="number" min="0" max="255" value="0">
          </label>
          <button id="copyButton" type="button" class="primary">Copy Output</button>
        </div>
        <div id="stats"></div>
      </div>

      <div class="panel">
        <h2>Edit</h2>
        <div class="button-row">
          <button id="undoButton" type="button">Undo Point</button>
          <button id="newPolygonButton" type="button">New Polygon</button>
          <button id="clearButton" type="button" class="danger">Clear</button>
        </div>
      </div>

      <div class="panel">
        <h2>Param YAML</h2>
        <label>
          Path or file URL
          <input id="paramPath" type="text" spellcheck="false">
        </label>
        <div class="button-row">
          <button id="loadParamButton" type="button">Load Param</button>
          <button id="applyParamButton" type="button">Apply Loaded</button>
          <button id="saveParamButton" type="button" class="primary">Save Param</button>
        </div>
        <div id="paramStatus" class="status" style="margin-top: 8px;"></div>
      </div>

      <div class="panel">
        <h2>Evidence</h2>
        <label>
          Output folder
          <input id="outputDir" type="text" spellcheck="false">
        </label>
        <label class="check"><input id="saveOverlayVariant" type="checkbox" checked> Overlay</label>
        <label class="check"><input id="saveFilledVariant" type="checkbox" checked> Filled preview</label>
        <label class="check"><input id="saveOutlineVariant" type="checkbox" checked> Outline</label>
        <div class="button-row">
          <button id="saveEvidenceButton" type="button" class="primary">Save Evidence PNGs</button>
        </div>
        <div id="evidenceStatus" class="status" style="margin-top: 8px;"></div>
      </div>

      <div class="panel">
        <h2>Frame Cache</h2>
        <label>
          Cache folder
          <input id="cacheDir" type="text" spellcheck="false">
        </label>
        <div class="button-row">
          <button id="saveCacheButton" type="button">Save Current Frame</button>
        </div>
        <div id="cacheStatus" class="status" style="margin-top: 8px;"></div>
      </div>

      <div class="panel">
        <h2>Import Points</h2>
        <textarea id="importText" spellcheck="false">[0.8, 0.0, 1.0, 0.0, 1.0, 1.0, 0.8, 1.0]</textarea>
        <div class="button-row">
          <button id="importButton" type="button">Import</button>
          <button id="sampleLeftButton" type="button">Left Strip</button>
          <button id="sampleRightButton" type="button">Right Strip</button>
        </div>
      </div>

      <div class="panel">
        <h2>Output</h2>
        <label>
          ROS parameter snippet
          <textarea id="paramOutput" readonly spellcheck="false"></textarea>
        </label>
        <label style="margin-top: 10px;">
          polygons YAML
          <textarea id="yamlOutput" readonly spellcheck="false"></textarea>
        </label>
      </div>
    </aside>
  </main>

  <script>
    const canvas = document.getElementById("maskCanvas");
    const ctx = canvas.getContext("2d");
    const streamSelect = document.getElementById("streamSelect");
    const statusEl = document.getElementById("status");
    const statsEl = document.getElementById("stats");
    const cameraIdEl = document.getElementById("cameraId");
    const normalizedEl = document.getElementById("normalized");
    const importTextEl = document.getElementById("importText");
    const paramOutputEl = document.getElementById("paramOutput");
    const yamlOutputEl = document.getElementById("yamlOutput");
    const fillBEl = document.getElementById("fillB");
    const fillGEl = document.getElementById("fillG");
    const fillREl = document.getElementById("fillR");
    const liveToggle = document.getElementById("liveToggle");
    const paramPathEl = document.getElementById("paramPath");
    const paramStatusEl = document.getElementById("paramStatus");
    const outputDirEl = document.getElementById("outputDir");
    const evidenceStatusEl = document.getElementById("evidenceStatus");
    const cacheDirEl = document.getElementById("cacheDir");
    const cacheStatusEl = document.getElementById("cacheStatus");
    const saveOverlayVariantEl = document.getElementById("saveOverlayVariant");
    const saveFilledVariantEl = document.getElementById("saveFilledVariant");
    const saveOutlineVariantEl = document.getElementById("saveOutlineVariant");

    const state = {
      image: null,
      streams: [],
      selectedKey: "",
      loadedParam: null,
      polygons: [[]],
      activePolygon: 0,
      drag: null,
      hover: null,
      lastFrameOk: false,
    };

    function clamp(value, min, max) { return Math.max(min, Math.min(max, value)); }
    function round(value, digits) {
      const scale = 10 ** digits;
      return Math.round(value * scale) / scale;
    }
    function activePoints() {
      if (!state.polygons[state.activePolygon]) state.polygons[state.activePolygon] = [];
      return state.polygons[state.activePolygon];
    }
    function validPolygons() { return state.polygons.filter((points) => points.length >= 3); }
    function currentStream() { return state.streams.find((stream) => stream.key === state.selectedKey); }
    function imageSizeText() {
      if (!state.image) return "no frame";
      return `${state.image.naturalWidth} x ${state.image.naturalHeight}`;
    }
    function setStatus(text) { statusEl.textContent = text; }
    function setParamStatus(text) { paramStatusEl.textContent = text; }
    function setEvidenceStatus(text) { evidenceStatusEl.textContent = text; }
    function setCacheStatus(text) { cacheStatusEl.textContent = text; }

    function pointerToImagePoint(event) {
      const rect = canvas.getBoundingClientRect();
      const x = (event.clientX - rect.left) * canvas.width / rect.width;
      const y = (event.clientY - rect.top) * canvas.height / rect.height;
      return { x: clamp(x, 0, canvas.width - 1), y: clamp(y, 0, canvas.height - 1) };
    }
    function displayThreshold() {
      const rect = canvas.getBoundingClientRect();
      return 11 * canvas.width / Math.max(1, rect.width);
    }
    function findVertex(point) {
      const threshold = displayThreshold();
      let best = null;
      let bestDist = Number.POSITIVE_INFINITY;
      state.polygons.forEach((polygon, polygonIndex) => {
        polygon.forEach((vertex, vertexIndex) => {
          const dist = Math.hypot(vertex.x - point.x, vertex.y - point.y);
          if (dist < threshold && dist < bestDist) {
            best = { polygonIndex, vertexIndex };
            bestDist = dist;
          }
        });
      });
      return best;
    }

    function drawPolygon(points, index) {
      if (points.length === 0) return;
      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      points.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
      if (points.length >= 3) {
        ctx.closePath();
        ctx.fillStyle = index === state.activePolygon ? "rgba(0, 108, 103, 0.32)" : "rgba(255, 138, 76, 0.24)";
        ctx.fill();
      }
      ctx.lineWidth = Math.max(2, canvas.width / 900);
      ctx.strokeStyle = index === state.activePolygon ? "#00a991" : "#ff9f43";
      ctx.stroke();
      points.forEach((point, vertexIndex) => {
        const isHover = state.hover &&
          state.hover.polygonIndex === index &&
          state.hover.vertexIndex === vertexIndex;
        ctx.beginPath();
        ctx.arc(point.x, point.y, isHover ? 8 : 6, 0, Math.PI * 2);
        ctx.fillStyle = isHover ? "#ffffff" : "#ffd166";
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = "#101828";
        ctx.stroke();
      });
    }

    function drawPlaceholder(text) {
      ctx.fillStyle = "#182230";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#d0d5dd";
      ctx.font = "24px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    }
    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      if (state.image) {
        ctx.drawImage(state.image, 0, 0, canvas.width, canvas.height);
      } else {
        drawPlaceholder("Waiting for frame");
      }
      state.polygons.forEach(drawPolygon);
      updateOutput();
      updateStats();
    }

    function outputPoints(points) {
      const normalized = normalizedEl.checked;
      const width = canvas.width;
      const height = canvas.height;
      return points.flatMap((point) => {
        if (normalized) return [round(point.x / width, 6), round(point.y / height, 6)];
        return [round(point.x, 2), round(point.y, 2)];
      });
    }
    function formatArray(values) {
      return `[${values.map((value) => Number.isInteger(value) ? value.toString() : value.toFixed(6).replace(/0+$/, "").replace(/\.$/, "")).join(", ")}]`;
    }
    function updateOutput() {
      const cameraId = clamp(Number.parseInt(cameraIdEl.value || "0", 10), 0, 99);
      const normalized = normalizedEl.checked ? "true" : "false";
      const polygons = validPolygons();
      const first = polygons[0] || [];
      const firstMask = first.length ? formatArray(outputPoints(first)) : "[]";
      const fill = [
        clamp(Number.parseInt(fillBEl.value || "0", 10), 0, 255),
        clamp(Number.parseInt(fillGEl.value || "0", 10), 0, 255),
        clamp(Number.parseInt(fillREl.value || "0", 10), 0, 255),
      ];
      paramOutputEl.value =
        `ego_mask.fill_value_bgr: [${fill.join(", ")}]\n` +
        `camera_${cameraId}_mask:\n` +
        `  enable: ${first.length ? "true" : "false"}\n` +
        `  mask: ${firstMask}\n` +
        `  normalized: ${normalized}\n`;
      if (polygons.length === 0) {
        yamlOutputEl.value = "polygons: []\n";
      } else {
        const yamlLines = ["polygons:"];
        polygons.forEach((points) => {
          yamlLines.push(`  - points: ${formatArray(outputPoints(points))}`);
          yamlLines.push(`    normalized: ${normalized}`);
        });
        yamlOutputEl.value = `${yamlLines.join("\n")}\n`;
      }
    }
    function setFillBgr(fill) {
      if (!Array.isArray(fill) || fill.length < 3) return;
      fillBEl.value = Math.round(clamp(Number(fill[0]), 0, 255));
      fillGEl.value = Math.round(clamp(Number(fill[1]), 0, 255));
      fillREl.value = Math.round(clamp(Number(fill[2]), 0, 255));
    }
    function pointsFromMask(values, normalized) {
      const points = [];
      if (!Array.isArray(values)) return points;
      for (let i = 0; i + 1 < values.length; i += 2) {
        const x = normalized ? Number(values[i]) * canvas.width : Number(values[i]);
        const y = normalized ? Number(values[i + 1]) * canvas.height : Number(values[i + 1]);
        points.push({ x: clamp(x, 0, canvas.width - 1), y: clamp(y, 0, canvas.height - 1) });
      }
      return points;
    }
    function applyLoadedParamForCamera() {
      if (!state.loadedParam) {
        setParamStatus("no param loaded");
        return;
      }
      const cameraId = String(clamp(Number.parseInt(cameraIdEl.value || "0", 10), 0, 99));
      const mask = state.loadedParam.masks[cameraId];
      setFillBgr(state.loadedParam.fill_value_bgr);
      if (!mask || !mask.enable || !Array.isArray(mask.mask) || mask.mask.length < 6) {
        state.polygons = [[]];
        state.activePolygon = 0;
        setParamStatus(`camera_${cameraId}_mask is disabled or empty`);
        draw();
        return;
      }
      normalizedEl.checked = Boolean(mask.normalized);
      state.polygons = [pointsFromMask(mask.mask, mask.normalized)];
      state.activePolygon = 0;
      setParamStatus(`loaded camera_${cameraId}_mask from param`);
      draw();
    }
    async function loadParam() {
      const path = paramPathEl.value.trim();
      if (!path) {
        setParamStatus("param path is empty");
        return;
      }
      const response = await fetch(`/api/param?path=${encodeURIComponent(path)}`);
      const data = await response.json();
      if (!response.ok) {
        setParamStatus(data.error || "failed to load param");
        return;
      }
      state.loadedParam = data;
      paramPathEl.value = data.path;
      setFillBgr(data.fill_value_bgr);
      setParamStatus(`loaded ${data.path}`);
      applyLoadedParamForCamera();
    }
    async function saveParam() {
      const path = paramPathEl.value.trim();
      if (!path) {
        setParamStatus("param path is empty");
        return;
      }
      const polygons = validPolygons();
      const cameraId = clamp(Number.parseInt(cameraIdEl.value || "0", 10), 0, 99);
      const payload = {
        path,
        camera_id: cameraId,
        enable: polygons.length > 0,
        mask: polygons.length > 0 ? outputPoints(polygons[0]) : [],
        normalized: normalizedEl.checked,
        fill_value_bgr: [
          clamp(Number.parseInt(fillBEl.value || "0", 10), 0, 255),
          clamp(Number.parseInt(fillGEl.value || "0", 10), 0, 255),
          clamp(Number.parseInt(fillREl.value || "0", 10), 0, 255),
        ],
      };
      const response = await fetch("/api/param", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await response.json();
      if (!response.ok) {
        setParamStatus(data.error || "failed to save param");
        return;
      }
      state.loadedParam = data;
      paramPathEl.value = data.path;
      setParamStatus(`saved camera_${cameraId}_mask to ${data.path}`);
    }
    function drawEvidencePolygons(targetCtx, variant) {
      validPolygons().forEach((points) => {
        targetCtx.beginPath();
        targetCtx.moveTo(points[0].x, points[0].y);
        points.slice(1).forEach((point) => targetCtx.lineTo(point.x, point.y));
        targetCtx.closePath();
        if (variant === "overlay") {
          targetCtx.fillStyle = "rgba(0, 169, 145, 0.36)";
          targetCtx.fill();
          targetCtx.lineWidth = Math.max(3, canvas.width / 700);
          targetCtx.strokeStyle = "rgba(255, 209, 102, 0.95)";
          targetCtx.stroke();
        } else if (variant === "filled") {
          const fill = [
            clamp(Number.parseInt(fillBEl.value || "0", 10), 0, 255),
            clamp(Number.parseInt(fillGEl.value || "0", 10), 0, 255),
            clamp(Number.parseInt(fillREl.value || "0", 10), 0, 255),
          ];
          targetCtx.fillStyle = `rgb(${fill[2]}, ${fill[1]}, ${fill[0]})`;
          targetCtx.fill();
        } else if (variant === "outline") {
          targetCtx.lineWidth = Math.max(4, canvas.width / 600);
          targetCtx.strokeStyle = "rgba(255, 209, 102, 1.0)";
          targetCtx.stroke();
        }
      });
    }
    function evidenceDataUrl(variant) {
      if (!state.image) return "";
      const evidenceCanvas = document.createElement("canvas");
      evidenceCanvas.width = canvas.width;
      evidenceCanvas.height = canvas.height;
      const evidenceCtx = evidenceCanvas.getContext("2d");
      evidenceCtx.drawImage(state.image, 0, 0, evidenceCanvas.width, evidenceCanvas.height);
      drawEvidencePolygons(evidenceCtx, variant);
      return evidenceCanvas.toDataURL("image/png");
    }
    async function saveEvidence() {
      const outputDir = outputDirEl.value.trim();
      if (!outputDir) {
        setEvidenceStatus("output folder is empty");
        return;
      }
      if (!state.image) {
        setEvidenceStatus("no frame loaded");
        return;
      }
      const variants = [];
      if (saveOverlayVariantEl.checked) variants.push("overlay");
      if (saveFilledVariantEl.checked) variants.push("filled");
      if (saveOutlineVariantEl.checked) variants.push("outline");
      if (variants.length === 0) {
        setEvidenceStatus("select at least one evidence variant");
        return;
      }
      const payload = {
        output_dir: outputDir,
        camera_id: clamp(Number.parseInt(cameraIdEl.value || "0", 10), 0, 99),
        stream: currentStream() ? currentStream().image_topic : "",
        images: variants.map((variant) => ({ variant, image_data: evidenceDataUrl(variant) })),
      };
      const response = await fetch("/api/evidence", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await response.json();
      if (!response.ok) {
        setEvidenceStatus(data.error || "failed to save evidence");
        return;
      }
      setEvidenceStatus(`saved ${data.paths.length} files`);
    }
    async function saveCache() {
      const cacheDir = cacheDirEl.value.trim();
      if (!cacheDir) {
        setCacheStatus("cache folder is empty");
        return;
      }
      if (!state.selectedKey) {
        setCacheStatus("no stream selected");
        return;
      }
      const response = await fetch("/api/cache", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ key: state.selectedKey, cache_dir: cacheDir }),
      });
      const data = await response.json();
      if (!response.ok) {
        setCacheStatus(data.error || "failed to save cache");
        return;
      }
      setCacheStatus(`cached ${data.metadata_path}`);
    }
    function updateStats() {
      const stream = currentStream();
      const rows = [
        ["Frame", imageSizeText()],
        ["Stream", stream ? stream.image_topic : "none"],
        ["Type", stream ? stream.image_type : "none"],
        ["CameraInfo", stream && stream.has_camera_info ? stream.camera_info_topic : "not paired"],
        ["Mode", stream && stream.is_undistorted ? "undistorted" : "original/fallback"],
        ["Polygons", validPolygons().length.toString()],
        ["Active points", activePoints().length.toString()],
      ];
      statsEl.innerHTML = rows.map(([key, value]) => `<div class="stat"><span>${key}</span><span>${value}</span></div>`).join("");
    }

    async function refreshStreams() {
      const response = await fetch("/api/streams");
      state.streams = await response.json();
      const previous = state.selectedKey;
      streamSelect.innerHTML = "";
      state.streams.forEach((stream) => {
        const option = document.createElement("option");
        option.value = stream.key;
        const info = stream.has_camera_info ? "info" : "no info";
        const age = stream.last_frame_age_sec == null ? "no frame" : `${stream.last_frame_age_sec.toFixed(1)}s`;
        option.textContent = `${stream.image_topic} (${stream.image_type}, ${info}, ${age})`;
        streamSelect.appendChild(option);
      });
      if (state.streams.length === 0) {
        setStatus("no image topics discovered");
        state.selectedKey = "";
        draw();
        return;
      }
      state.selectedKey = state.streams.some((stream) => stream.key === previous)
        ? previous
        : state.streams[0].key;
      streamSelect.value = state.selectedKey;
      inferCameraId();
      setStatus(`${state.streams.length} image streams subscribed`);
      updateStats();
    }

    function inferCameraId() {
      const stream = currentStream();
      if (!stream) return;
      const match = stream.image_topic.match(/camera[_/\\-]?(\d+)/i);
      if (match) cameraIdEl.value = match[1];
      if (state.loadedParam) applyLoadedParamForCamera();
    }

    function loadFrame() {
      if (!state.selectedKey) return;
      const image = new Image();
      image.onload = () => {
        state.image = image;
        if (canvas.width !== image.naturalWidth || canvas.height !== image.naturalHeight) {
          canvas.width = image.naturalWidth;
          canvas.height = image.naturalHeight;
        }
        state.lastFrameOk = true;
        draw();
      };
      image.onerror = () => {
        state.lastFrameOk = false;
        draw();
      };
      image.src = `/api/frame?key=${encodeURIComponent(state.selectedKey)}&t=${Date.now()}`;
    }

    function importPoints(text) {
      const normalizedMatch = text.match(/normalized\s*:\s*(true|false)/i);
      if (normalizedMatch) normalizedEl.checked = normalizedMatch[1].toLowerCase() === "true";
      const arrayMatch =
        text.match(/(?:mask|points)\s*:\s*\[([^\]]*)\]/i) || text.match(/\[([^\]]*)\]/);
      const pointText = arrayMatch ? arrayMatch[1] : text;
      const values = (pointText.match(/-?\d+(?:\.\d+)?(?:e[-+]?\d+)?/gi) || []).map(Number);
      if (values.length < 6 || values.length % 2 !== 0) {
        setStatus("import needs at least 3 points");
        return;
      }
      const isNormalized = normalizedEl.checked;
      const points = [];
      for (let i = 0; i < values.length; i += 2) {
        const x = isNormalized ? values[i] * canvas.width : values[i];
        const y = isNormalized ? values[i + 1] * canvas.height : values[i + 1];
        points.push({ x: clamp(x, 0, canvas.width - 1), y: clamp(y, 0, canvas.height - 1) });
      }
      state.polygons[state.activePolygon] = points;
      setStatus("imported points");
      draw();
    }

    canvas.addEventListener("pointerdown", (event) => {
      if (!state.image) return;
      const point = pointerToImagePoint(event);
      const vertex = findVertex(point);
      if (vertex) {
        state.activePolygon = vertex.polygonIndex;
        state.drag = vertex;
      } else {
        activePoints().push(point);
      }
      draw();
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!state.image) return;
      const point = pointerToImagePoint(event);
      if (state.drag) state.polygons[state.drag.polygonIndex][state.drag.vertexIndex] = point;
      state.hover = findVertex(point);
      draw();
    });
    canvas.addEventListener("pointerup", () => { state.drag = null; });
    canvas.addEventListener("pointerleave", () => { state.drag = null; state.hover = null; draw(); });

    streamSelect.addEventListener("change", () => {
      state.selectedKey = streamSelect.value;
      state.image = null;
      state.polygons = [[]];
      state.activePolygon = 0;
      inferCameraId();
      loadFrame();
    });
    document.getElementById("refreshButton").addEventListener("click", refreshStreams);
    document.getElementById("freezeButton").addEventListener("click", () => {
      liveToggle.checked = false;
      loadFrame();
      setStatus("frame frozen");
    });
    document.getElementById("undoButton").addEventListener("click", () => { activePoints().pop(); draw(); });
    document.getElementById("newPolygonButton").addEventListener("click", () => {
      state.polygons.push([]);
      state.activePolygon = state.polygons.length - 1;
      draw();
    });
    document.getElementById("clearButton").addEventListener("click", () => {
      state.polygons = [[]];
      state.activePolygon = 0;
      draw();
    });
    document.getElementById("importButton").addEventListener("click", () => importPoints(importTextEl.value));
    document.getElementById("loadParamButton").addEventListener("click", loadParam);
    document.getElementById("applyParamButton").addEventListener("click", applyLoadedParamForCamera);
    document.getElementById("saveParamButton").addEventListener("click", saveParam);
    document.getElementById("saveEvidenceButton").addEventListener("click", saveEvidence);
    document.getElementById("saveCacheButton").addEventListener("click", saveCache);
    document.getElementById("sampleLeftButton").addEventListener("click", () => {
      normalizedEl.checked = true;
      importTextEl.value = "[0.0, 0.0, 0.2, 0.0, 0.2, 1.0, 0.0, 1.0]";
      importPoints(importTextEl.value);
    });
    document.getElementById("sampleRightButton").addEventListener("click", () => {
      normalizedEl.checked = true;
      importTextEl.value = "[0.8, 0.0, 1.0, 0.0, 1.0, 1.0, 0.8, 1.0]";
      importPoints(importTextEl.value);
    });
    document.getElementById("copyButton").addEventListener("click", async () => {
      const text = `${paramOutputEl.value}\n${yamlOutputEl.value}`;
      try {
        await navigator.clipboard.writeText(text);
        setStatus("copied output");
      } catch (error) {
        paramOutputEl.focus();
        paramOutputEl.select();
        setStatus("copy unavailable");
      }
    });
    [cameraIdEl, normalizedEl, fillBEl, fillGEl, fillREl].forEach((element) => {
      element.addEventListener("input", draw);
      element.addEventListener("change", draw);
    });
    cameraIdEl.addEventListener("change", () => {
      if (state.loadedParam) applyLoadedParamForCamera();
    });

    setInterval(() => {
      if (liveToggle.checked && !state.drag) loadFrame();
    }, 650);
    setInterval(refreshStreams, 3000);

    draw();
    fetch("/api/config").then((response) => response.json()).then((config) => {
      paramPathEl.value = config.default_param_path || "";
      outputDirEl.value = config.default_output_dir || "";
      cacheDirEl.value = config.default_cache_dir || "";
    });
    refreshStreams().then(loadFrame);
  </script>
</body>
</html>
"""


IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"


@dataclass
class StreamState:
    key: str
    image_topic: str
    image_type: str
    compressed: bool
    camera_info_topic: Optional[str] = None
    camera_info: Optional[CameraInfo] = None
    latest_bgr: Optional[np.ndarray] = None
    latest_stamp_sec: Optional[float] = None
    latest_encoding: str = ""
    latest_error: str = ""
    map_signature: str = ""
    map_x: Optional[np.ndarray] = None
    map_y: Optional[np.ndarray] = None
    lock: threading.Lock = field(default_factory=threading.Lock)


class CameraMaskDesignerNode(Node):
    def __init__(
        self, jpeg_quality: int, default_param_path: str, default_output_dir: str,
        default_cache_dir: str
    ) -> None:
        super().__init__("streampetr_camera_mask_designer")
        self._jpeg_quality = int(max(1, min(100, jpeg_quality)))
        self._default_param_path = default_param_path
        self._default_output_dir = default_output_dir
        self._default_cache_dir = default_cache_dir
        self._streams: dict[str, StreamState] = {}
        self._camera_infos: dict[str, CameraInfo] = {}
        self._subscriptions_by_topic: set[str] = set()
        self._lock = threading.Lock()

        self._discover_topics()
        self.create_timer(2.0, self._discover_topics)

    def config_summary(self) -> dict[str, str]:
        return {
            "default_param_path": self._default_param_path,
            "default_output_dir": self._default_output_dir,
            "default_cache_dir": self._default_cache_dir,
        }

    def _discover_topics(self) -> None:
        topic_map = self.get_topic_names_and_types()
        camera_info_topics = sorted(
            topic for topic, types in topic_map if CAMERA_INFO_TYPE in set(types)
        )

        for topic in camera_info_topics:
            if topic not in self._subscriptions_by_topic:
                self.create_subscription(
                    CameraInfo, topic, self._make_camera_info_callback(topic), qos_profile_sensor_data
                )
                self._subscriptions_by_topic.add(topic)

        for topic, types in topic_map:
            type_set = set(types)
            if "sensor_msgs/msg/Image" in type_set:
                self._ensure_image_subscription(topic, "sensor_msgs/msg/Image", camera_info_topics)
            if "sensor_msgs/msg/CompressedImage" in type_set:
                self._ensure_image_subscription(
                    topic, "sensor_msgs/msg/CompressedImage", camera_info_topics
                )

        self._refresh_stream_camera_info_topics(camera_info_topics)

    def _ensure_image_subscription(
        self, topic: str, image_type: str, camera_info_topics: list[str]
    ) -> None:
        key = self._make_stream_key(topic, image_type)
        compressed = image_type == "sensor_msgs/msg/CompressedImage"
        with self._lock:
            if key not in self._streams:
                self._streams[key] = StreamState(
                    key=key,
                    image_topic=topic,
                    image_type=image_type,
                    compressed=compressed,
                    camera_info_topic=self._choose_camera_info_topic(topic, camera_info_topics),
                )

        if topic in self._subscriptions_by_topic:
            return

        if compressed:
            self.create_subscription(
                CompressedImage, topic, self._make_image_callback(key), qos_profile_sensor_data
            )
        else:
            self.create_subscription(
                Image, topic, self._make_image_callback(key), qos_profile_sensor_data
            )
        self._subscriptions_by_topic.add(topic)
        self.get_logger().info(f"subscribed image stream: {topic} ({image_type})")

    def _refresh_stream_camera_info_topics(self, camera_info_topics: list[str]) -> None:
        with self._lock:
            streams = list(self._streams.values())
        for stream in streams:
            chosen = self._choose_camera_info_topic(stream.image_topic, camera_info_topics)
            with stream.lock:
                if chosen and stream.camera_info_topic != chosen:
                    stream.camera_info_topic = chosen
                    stream.camera_info = self._camera_infos.get(chosen)
                    stream.map_signature = ""
                    stream.map_x = None
                    stream.map_y = None

    @staticmethod
    def _make_stream_key(topic: str, image_type: str) -> str:
        digest = hashlib.sha1(f"{topic}|{image_type}".encode("utf-8")).hexdigest()[:12]
        return digest

    @staticmethod
    def _base_image_topic(topic: str) -> str:
        return topic[: -len("/compressed")] if topic.endswith("/compressed") else topic

    @classmethod
    def _candidate_info_topics(cls, image_topic: str) -> list[str]:
        base = cls._base_image_topic(image_topic)
        candidates = []
        if "/" in base:
            parent = base.rsplit("/", 1)[0]
            candidates.append(f"{parent}/camera_info")
        for image_name in ("image_raw", "image_rect", "image_rect_color", "image_color", "image"):
            suffix = f"/{image_name}"
            if base.endswith(suffix):
                candidates.append(base[: -len(suffix)] + "/camera_info")
        return list(dict.fromkeys(candidates))

    @classmethod
    def _choose_camera_info_topic(
        cls, image_topic: str, camera_info_topics: list[str]
    ) -> Optional[str]:
        for candidate in cls._candidate_info_topics(image_topic):
            if candidate in camera_info_topics:
                return candidate

        base = cls._base_image_topic(image_topic)
        best_topic = None
        best_score = -1
        for info_topic in camera_info_topics:
            score = len(_common_prefix(base, info_topic))
            if score > best_score:
                best_topic = info_topic
                best_score = score
        return best_topic if best_score > 1 else None

    def _make_camera_info_callback(self, topic: str) -> Callable[[CameraInfo], None]:
        def callback(msg: CameraInfo) -> None:
            self._camera_infos[topic] = msg
            with self._lock:
                streams = list(self._streams.values())
            for stream in streams:
                if stream.camera_info_topic == topic:
                    with stream.lock:
                        stream.camera_info = msg
        return callback

    def _make_image_callback(self, key: str) -> Callable[[Image | CompressedImage], None]:
        def callback(msg: Image | CompressedImage) -> None:
            with self._lock:
                stream = self._streams.get(key)
            if stream is None:
                return

            try:
                if stream.compressed:
                    image_bgr = _compressed_image_to_bgr(msg)
                    encoding = "compressed/bgr8"
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                else:
                    image_bgr = _image_msg_to_bgr(msg)
                    encoding = getattr(msg, "encoding", "bgr8")
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            except Exception as error:  # noqa: BLE001 - surface conversion failures to UI.
                with stream.lock:
                    stream.latest_error = str(error)
                return

            with stream.lock:
                stream.latest_bgr = image_bgr
                stream.latest_stamp_sec = stamp if stamp > 0.0 else time.time()
                stream.latest_encoding = encoding
                stream.latest_error = ""

        return callback

    def streams_summary(self) -> list[dict[str, object]]:
        now = time.time()
        with self._lock:
            streams = sorted(self._streams.values(), key=lambda item: item.image_topic)

        summaries = []
        for stream in streams:
            with stream.lock:
                age = None
                if stream.latest_stamp_sec is not None:
                    age = max(0.0, now - stream.latest_stamp_sec)
                width = None
                height = None
                if stream.latest_bgr is not None:
                    height, width = stream.latest_bgr.shape[:2]
                summaries.append(
                    {
                        "key": stream.key,
                        "image_topic": stream.image_topic,
                        "image_type": "compressed" if stream.compressed else "raw",
                        "camera_info_topic": stream.camera_info_topic,
                        "has_camera_info": stream.camera_info is not None,
                        "is_undistorted": stream.camera_info is not None and not stream.latest_error,
                        "last_frame_age_sec": age,
                        "width": width,
                        "height": height,
                        "encoding": stream.latest_encoding,
                        "error": stream.latest_error,
                    }
                )
        return summaries

    def encoded_frame(self, key: str) -> tuple[Optional[bytes], str]:
        image_bgr, error = self._frame_bgr(key)
        if image_bgr is None:
            return None, error

        success, encoded = cv2.imencode(
            ".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        )
        if not success:
            return None, "failed to encode frame"
        return encoded.tobytes(), ""

    def save_cached_frame(self, key: str, cache_dir_text: str) -> dict[str, str]:
        image_bgr, error = self._frame_bgr(key)
        if image_bgr is None:
            raise RuntimeError(error)

        stream = self._stream_for_key(key)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_stream = re.sub(r"[^A-Za-z0-9_.-]+", "_", stream.image_topic.strip("_"))[:80]
        stem = f"{timestamp}_{safe_stream}_{key}"
        cache_dir = _resolve_local_path(cache_dir_text)
        cache_dir.mkdir(parents=True, exist_ok=True)
        image_path = cache_dir / f"{stem}.jpg"
        metadata_path = cache_dir / f"{stem}.json"
        if not cv2.imwrite(str(image_path), image_bgr):
            raise RuntimeError(f"failed to write cached frame: {image_path}")

        metadata = {
            "key": f"cached_{key}_{timestamp}",
            "image_topic": stream.image_topic,
            "image_type": stream.image_type,
            "camera_info_topic": stream.camera_info_topic,
            "has_camera_info": stream.camera_info is not None,
            "is_undistorted": stream.camera_info is not None,
            "width": int(image_bgr.shape[1]),
            "height": int(image_bgr.shape[0]),
            "encoding": "bgr8-jpeg",
            "image_file": image_path.name,
        }
        if stream.camera_info is not None:
            metadata.update(
                {
                    "camera_info_k": list(stream.camera_info.k),
                    "camera_info_d": list(stream.camera_info.d),
                    "camera_info_p": list(stream.camera_info.p),
                    "camera_info_width": int(stream.camera_info.width),
                    "camera_info_height": int(stream.camera_info.height),
                    "camera_frame_id": stream.camera_info.header.frame_id,
                }
            )
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return {"image_path": str(image_path), "metadata_path": str(metadata_path)}

    def _stream_for_key(self, key: str) -> StreamState:
        with self._lock:
            stream = self._streams.get(key)
        if stream is None:
            raise RuntimeError("unknown stream")
        return stream

    def _frame_bgr(self, key: str) -> tuple[Optional[np.ndarray], str]:
        try:
            stream = self._stream_for_key(key)
        except RuntimeError as error:
            return None, str(error)
        with stream.lock:
            if stream.latest_bgr is None:
                return None, "no frame received yet"
            image_bgr = stream.latest_bgr.copy()
            camera_info = stream.camera_info

        try:
            if camera_info is not None:
                image_bgr = self._undistort(stream, image_bgr, camera_info)
        except Exception as error:  # noqa: BLE001 - return UI-readable error.
            return None, str(error)
        return image_bgr, ""

    def _undistort(
        self, stream: StreamState, image_bgr: np.ndarray, camera_info: CameraInfo
    ) -> np.ndarray:
        width = int(camera_info.width)
        height = int(camera_info.height)
        if image_bgr.shape[1] != width or image_bgr.shape[0] != height:
            raise RuntimeError(
                "image size does not match camera_info "
                f"({image_bgr.shape[1]}x{image_bgr.shape[0]} != {width}x{height})"
            )

        signature = _camera_info_signature(camera_info)
        with stream.lock:
            if stream.map_signature != signature or stream.map_x is None or stream.map_y is None:
                k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
                d = np.array(camera_info.d, dtype=np.float64).reshape(1, len(camera_info.d))
                p = np.array(
                    [
                        [camera_info.p[0], camera_info.p[1], camera_info.p[2]],
                        [camera_info.p[4], camera_info.p[5], camera_info.p[6]],
                        [camera_info.p[8], camera_info.p[9], camera_info.p[10]],
                    ],
                    dtype=np.float64,
                )
                stream.map_x, stream.map_y = cv2.initUndistortRectifyMap(
                    k, d, None, p, (width, height), cv2.CV_32FC1
                )
                stream.map_signature = signature
            map_x = stream.map_x
            map_y = stream.map_y

        return cv2.remap(image_bgr, map_x, map_y, interpolation=cv2.INTER_LINEAR)


class SimpleLogger:
    def debug(self, message: str) -> None:
        print(f"[DEBUG] {message}")

    def info(self, message: str) -> None:
        print(f"[INFO] {message}")

    def warn(self, message: str) -> None:
        print(f"[WARN] {message}")


class CachedFrameProvider:
    def __init__(
        self, cache_dir: str, default_param_path: str, default_output_dir: str,
        default_cache_dir: str
    ) -> None:
        self._cache_dir = _resolve_local_path(cache_dir)
        self._default_param_path = default_param_path
        self._default_output_dir = default_output_dir
        self._default_cache_dir = default_cache_dir
        self._logger = SimpleLogger()

    def get_logger(self) -> SimpleLogger:
        return self._logger

    def config_summary(self) -> dict[str, str]:
        return {
            "default_param_path": self._default_param_path,
            "default_output_dir": self._default_output_dir,
            "default_cache_dir": self._default_cache_dir,
        }

    def streams_summary(self) -> list[dict[str, object]]:
        summaries = []
        for metadata_path in sorted(self._cache_dir.glob("*.json")):
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                image_path = self._cache_dir / str(metadata.get("image_file", ""))
                if not image_path.exists():
                    continue
                summaries.append(
                    {
                        "key": str(metadata.get("key", metadata_path.stem)),
                        "image_topic": str(metadata.get("image_topic", metadata_path.stem)),
                        "image_type": "cached",
                        "camera_info_topic": metadata.get("camera_info_topic"),
                        "has_camera_info": bool(metadata.get("has_camera_info", False)),
                        "is_undistorted": bool(metadata.get("is_undistorted", True)),
                        "last_frame_age_sec": 0.0,
                        "width": metadata.get("width"),
                        "height": metadata.get("height"),
                        "encoding": metadata.get("encoding", "bgr8-jpeg"),
                        "error": "",
                    }
                )
            except Exception as error:  # noqa: BLE001 - keep bad cache entries non-fatal.
                self._logger.warn(f"failed to read cache metadata {metadata_path}: {error}")
        return summaries

    def encoded_frame(self, key: str) -> tuple[Optional[bytes], str]:
        for metadata_path in sorted(self._cache_dir.glob("*.json")):
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            if str(metadata.get("key", metadata_path.stem)) != key:
                continue
            image_path = self._cache_dir / str(metadata.get("image_file", ""))
            if not image_path.exists():
                return None, f"cached image missing: {image_path}"
            return image_path.read_bytes(), ""
        return None, "unknown cached frame"

    def save_cached_frame(self, key: str, cache_dir_text: str) -> dict[str, str]:
        raise RuntimeError("cache saving is unavailable in offline cache mode")


class DesignerHttpServer(ThreadingHTTPServer):
    def __init__(
        self, server_address: tuple[str, int], handler_class: type[BaseHTTPRequestHandler],
        node: object
    ) -> None:
        super().__init__(server_address, handler_class)
        self.node = node


class DesignerRequestHandler(BaseHTTPRequestHandler):
    server: DesignerHttpServer

    def log_message(self, fmt: str, *args: object) -> None:
        self.server.node.get_logger().debug(fmt % args)

    def do_GET(self) -> None:  # noqa: N802 - stdlib API.
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if parsed.path == "/api/config":
            payload = json.dumps(self.server.node.config_summary()).encode("utf-8")
            self._send_bytes(payload, "application/json")
            return
        if parsed.path == "/api/streams":
            payload = json.dumps(self.server.node.streams_summary()).encode("utf-8")
            self._send_bytes(payload, "application/json")
            return
        if parsed.path == "/api/param":
            params = parse_qs(parsed.query)
            path = params.get("path", [""])[0]
            try:
                payload = json.dumps(load_param_file(path)).encode("utf-8")
            except Exception as error:  # noqa: BLE001 - return UI-readable error.
                self._send_error(HTTPStatus.BAD_REQUEST, str(error))
                return
            self._send_bytes(payload, "application/json")
            return
        if parsed.path == "/api/frame":
            params = parse_qs(parsed.query)
            key = params.get("key", [""])[0]
            payload, error = self.server.node.encoded_frame(key)
            if payload is None:
                self._send_error(HTTPStatus.NOT_FOUND, error)
                return
            self._send_bytes(payload, "image/jpeg", cache=False)
            return
        self._send_error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:  # noqa: N802 - stdlib API.
        parsed = urlparse(self.path)
        try:
            payload = self._read_json_body()
            if parsed.path == "/api/param":
                result = save_param_file(payload)
                self._send_bytes(json.dumps(result).encode("utf-8"), "application/json")
                return
            if parsed.path == "/api/evidence":
                result = save_evidence_png(payload)
                self._send_bytes(json.dumps(result).encode("utf-8"), "application/json")
                return
            if parsed.path == "/api/cache":
                result = self.server.node.save_cached_frame(
                    str(payload.get("key", "")), str(payload.get("cache_dir", ""))
                )
                self._send_bytes(json.dumps(result).encode("utf-8"), "application/json")
                return
        except Exception as error:  # noqa: BLE001 - return UI-readable error.
            self._send_error(HTTPStatus.BAD_REQUEST, str(error))
            return
        self._send_error(HTTPStatus.NOT_FOUND, "not found")

    def _read_json_body(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _send_bytes(self, payload: bytes, content_type: str, cache: bool = True) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        if not cache:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_error(self, status: HTTPStatus, message: str) -> None:
        payload = json.dumps({"error": message}).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def load_param_file(path_text: str) -> dict[str, object]:
    path = _resolve_local_path(path_text)
    text = path.read_text(encoding="utf-8")
    return {
        "path": str(path),
        "fill_value_bgr": _parse_fill_value_bgr(text),
        "masks": _parse_camera_masks(text),
    }


def save_param_file(payload: dict[str, object]) -> dict[str, object]:
    path = _resolve_local_path(str(payload.get("path", "")))
    text = path.read_text(encoding="utf-8") if path.exists() else "/**:\n  ros__parameters:\n"

    camera_id = int(payload.get("camera_id", 0))
    if camera_id < 0:
        raise RuntimeError("camera_id must be non-negative")

    mask = payload.get("mask", [])
    if not isinstance(mask, list):
        raise RuntimeError("mask must be a list")
    mask_values = [float(value) for value in mask]
    if mask_values and (len(mask_values) < 6 or len(mask_values) % 2 != 0):
        raise RuntimeError("mask must contain an even number of values and at least 3 points")

    fill = payload.get("fill_value_bgr", [0, 0, 0])
    if not isinstance(fill, list):
        raise RuntimeError("fill_value_bgr must be a list")
    fill_values = [int(max(0, min(255, round(float(value))))) for value in fill[:3]]
    while len(fill_values) < 3:
        fill_values.append(0)

    enable = bool(payload.get("enable", bool(mask_values)))
    normalized = bool(payload.get("normalized", True))
    text = _replace_fill_value_bgr(text, fill_values)
    text = _replace_camera_mask_block(text, camera_id, enable, mask_values, normalized)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return load_param_file(str(path))


def save_evidence_png(payload: dict[str, object]) -> dict[str, object]:
    output_dir = _resolve_local_path(str(payload.get("output_dir", "")))
    camera_id = int(payload.get("camera_id", 0))
    stream = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(payload.get("stream", "")).strip("_"))
    if not stream:
        stream = "camera"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)

    images = payload.get("images")
    if not isinstance(images, list):
        image_data = payload.get("image_data")
        images = [{"variant": "overlay", "image_data": image_data}]

    paths = []
    for item in images:
        if not isinstance(item, dict):
            continue
        variant = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(item.get("variant", "overlay")))
        image_data = str(item.get("image_data", ""))
        if not image_data.startswith("data:image/png;base64,"):
            raise RuntimeError("image_data must be a PNG data URL")
        filename = f"camera_{camera_id}_mask_{variant}_{timestamp}_{stream[:80]}.png"
        path = output_dir / filename
        path.write_bytes(base64.b64decode(image_data.split(",", 1)[1]))
        paths.append(str(path))
    if not paths:
        raise RuntimeError("no evidence images were provided")
    return {"paths": paths, "path": paths[0]}


def _resolve_local_path(path_text: str) -> Path:
    if not path_text.strip():
        raise RuntimeError("path is empty")
    parsed = urlparse(path_text)
    if parsed.scheme == "file":
        return Path(unquote(parsed.path)).expanduser().resolve()
    if parsed.scheme:
        raise RuntimeError(f"unsupported URL scheme: {parsed.scheme}")
    return Path(path_text).expanduser().resolve()


def _parse_fill_value_bgr(text: str) -> list[float]:
    match = re.search(r"(?m)^\s*ego_mask\.fill_value_bgr\s*:\s*\[([^\]]*)\]", text)
    if not match:
        return [0, 0, 0]
    values = _parse_number_list(match.group(1))
    while len(values) < 3:
        values.append(0)
    return values[:3]


def _parse_camera_masks(text: str) -> dict[str, dict[str, object]]:
    masks: dict[str, dict[str, object]] = {}
    for camera_id, block, _indent in _iter_camera_mask_blocks(text):
        enable_match = re.search(r"(?m)^\s*enable\s*:\s*(true|false)", block, re.IGNORECASE)
        normalized_match = re.search(
            r"(?m)^\s*normalized\s*:\s*(true|false)", block, re.IGNORECASE
        )
        mask_match = re.search(r"(?m)^\s*mask\s*:\s*\[([^\]]*)\]", block)
        masks[camera_id] = {
            "enable": _parse_bool(enable_match.group(1)) if enable_match else False,
            "mask": _parse_number_list(mask_match.group(1)) if mask_match else [],
            "normalized": _parse_bool(normalized_match.group(1)) if normalized_match else False,
        }
    return masks


def _parse_number_list(text: str) -> list[float]:
    return [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?(?:e[-+]?\d+)?", text, re.I)]


def _parse_bool(text: str) -> bool:
    return text.strip().lower() == "true"


def _replace_fill_value_bgr(text: str, fill_values: list[int]) -> str:
    line = f"ego_mask.fill_value_bgr: [{fill_values[0]}, {fill_values[1]}, {fill_values[2]}]"
    pattern = re.compile(r"(?m)^([ \t]*)ego_mask\.fill_value_bgr\s*:.*$")
    match = pattern.search(text)
    if match:
        return pattern.sub(rf"\1{line}", text, count=1)

    insert_indent = _default_parameter_indent(text)
    return _insert_after_ros_parameters(text, f"{insert_indent}{line}\n")


def _replace_camera_mask_block(
    text: str, camera_id: int, enable: bool, mask_values: list[float], normalized: bool
) -> str:
    span = _find_camera_mask_block_span(text, camera_id)
    indent = span[2] if span else _default_parameter_indent(text)
    block = _format_camera_mask_block(indent, camera_id, enable, mask_values, normalized)
    if span:
        return text[: span[0]] + block + text[span[1] :]
    return _insert_after_ros_parameters(text, block)


def _iter_camera_mask_blocks(text: str) -> list[tuple[str, str, str]]:
    blocks: list[tuple[str, str, str]] = []
    lines = text.splitlines(keepends=True)
    index = 0
    while index < len(lines):
        match = re.match(r"^([ \t]*)camera_(\d+)_mask\s*:\s*(?:#.*)?$", lines[index].rstrip("\n"))
        if not match:
            index += 1
            continue
        indent = match.group(1)
        camera_id = match.group(2)
        base_indent_len = len(indent.expandtabs(2))
        block_lines = []
        index += 1
        while index < len(lines):
            stripped = lines[index].strip()
            line_indent_len = len((lines[index])[: len(lines[index]) - len(lines[index].lstrip())].expandtabs(2))
            if stripped and line_indent_len <= base_indent_len:
                break
            block_lines.append(lines[index])
            index += 1
        blocks.append((camera_id, "".join(block_lines), indent))
    return blocks


def _find_camera_mask_block_span(text: str, camera_id: int) -> Optional[tuple[int, int, str]]:
    lines = text.splitlines(keepends=True)
    offset = 0
    index = 0
    while index < len(lines):
        line_start = offset
        line = lines[index]
        match = re.match(r"^([ \t]*)camera_(\d+)_mask\s*:\s*(?:#.*)?$", line.rstrip("\n"))
        offset += len(line)
        if not match:
            index += 1
            continue
        indent = match.group(1)
        base_indent_len = len(indent.expandtabs(2))
        block_end = offset
        index += 1
        while index < len(lines):
            stripped = lines[index].strip()
            line_indent_len = len((lines[index])[: len(lines[index]) - len(lines[index].lstrip())].expandtabs(2))
            if stripped and line_indent_len <= base_indent_len:
                break
            block_end += len(lines[index])
            index += 1
        if int(match.group(2)) == camera_id:
            return line_start, block_end, indent
        offset = block_end
    return None


def _format_camera_mask_block(
    indent: str, camera_id: int, enable: bool, mask_values: list[float], normalized: bool
) -> str:
    child_indent = indent + "  "
    return (
        f"{indent}camera_{camera_id}_mask:\n"
        f"{child_indent}enable: {'true' if enable else 'false'}\n"
        f"{child_indent}mask: {_format_number_array(mask_values)}\n"
        f"{child_indent}normalized: {'true' if normalized else 'false'}\n"
    )


def _format_number_array(values: list[float]) -> str:
    formatted = []
    for value in values:
        if abs(value - round(value)) < 1e-9:
            formatted.append(str(int(round(value))))
        else:
            formatted.append(f"{value:.6f}".rstrip("0").rstrip("."))
    return "[" + ", ".join(formatted) + "]"


def _default_parameter_indent(text: str) -> str:
    ros_match = re.search(r"(?m)^([ \t]*)ros__parameters\s*:\s*$", text)
    if ros_match:
        return ros_match.group(1) + "  "
    param_match = re.search(r"(?m)^([ \t]*)(?:ego_mask\.fill_value_bgr|camera_\d+_mask)\s*:", text)
    if param_match:
        return param_match.group(1)
    return "    "


def _insert_after_ros_parameters(text: str, block: str) -> str:
    ros_match = re.search(r"(?m)^.*ros__parameters\s*:\s*$", text)
    if ros_match:
        insert_at = ros_match.end()
        return text[:insert_at] + "\n" + block + text[insert_at:]
    if text and not text.endswith("\n"):
        text += "\n"
    return text + block


def _common_prefix(left: str, right: str) -> str:
    chars = []
    for left_char, right_char in zip(left, right):
        if left_char != right_char:
            break
        chars.append(left_char)
    return "".join(chars)


def _camera_info_signature(camera_info: CameraInfo) -> str:
    payload = json.dumps(
        {
            "width": int(camera_info.width),
            "height": int(camera_info.height),
            "k": list(camera_info.k),
            "d": list(camera_info.d),
            "p": list(camera_info.p),
        },
        sort_keys=True,
    )
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()


def _compressed_image_to_bgr(msg: CompressedImage) -> np.ndarray:
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    image_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError("failed to decode compressed image")
    return image_bgr


def _image_msg_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    if encoding not in channels_by_encoding:
        raise RuntimeError(f"unsupported image encoding for mask designer: {msg.encoding}")

    channels = channels_by_encoding[encoding]
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    row_size = width * channels
    if step < row_size:
        raise RuntimeError(
            f"invalid image step for {msg.encoding}: step={step}, expected at least {row_size}"
        )

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape((height, step))
    packed = rows[:, :row_size].reshape((height, width, channels))

    if encoding in ("bgr8", "8uc3"):
        return packed.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)
    if encoding in ("mono8", "8uc1"):
        return cv2.cvtColor(packed, cv2.COLOR_GRAY2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(packed, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(packed, cv2.COLOR_BGRA2BGR)

    raise RuntimeError(f"unsupported image encoding for mask designer: {msg.encoding}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve a browser UI for selecting live camera topics and drawing masks."
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8766, help="HTTP bind port")
    parser.add_argument(
        "--port-retries",
        type=int,
        default=10,
        help="Number of successive ports to try when the requested port is in use",
    )
    parser.add_argument(
        "--param-path",
        default="config/camera_streampetr.param.yaml",
        help="Default ROS parameter YAML path shown in the UI",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/streampetr_mask_evidence",
        help="Default folder for overlay evidence PNG files",
    )
    parser.add_argument(
        "--cache-dir",
        default="/tmp/streampetr_mask_frame_cache",
        help="Default folder for cached undistorted frames",
    )
    parser.add_argument(
        "--offline-cache-dir",
        default="",
        help="Serve cached frames from this folder without initializing ROS 2 subscriptions",
    )
    parser.add_argument("--jpeg-quality", type=int, default=85, help="Preview JPEG quality")
    return parser.parse_args()


def create_http_server(args: argparse.Namespace, node: object) -> DesignerHttpServer:
    for offset in range(max(1, args.port_retries + 1)):
        port = args.port + offset
        try:
            return DesignerHttpServer((args.host, port), DesignerRequestHandler, node)
        except OSError as error:
            if error.errno != errno.EADDRINUSE or offset >= args.port_retries:
                raise
            node.get_logger().warn(f"port {port} is in use; trying {port + 1}")

    raise RuntimeError("unreachable")


def main() -> None:
    args = parse_args()
    if args.offline_cache_dir:
        provider = CachedFrameProvider(
            cache_dir=args.offline_cache_dir,
            default_param_path=args.param_path,
            default_output_dir=args.output_dir,
            default_cache_dir=args.cache_dir,
        )
        server = create_http_server(args, provider)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        host, port = server.server_address[:2]
        provider.get_logger().info(f"mask designer UI: http://{host}:{port}/")
        provider.get_logger().info("offline cache mode: ROS 2 subscriptions are disabled")
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            pass
        finally:
            server.shutdown()
            server.server_close()
        return

    rclpy.init()
    node = CameraMaskDesignerNode(
        jpeg_quality=args.jpeg_quality,
        default_param_path=args.param_path,
        default_output_dir=args.output_dir,
        default_cache_dir=args.cache_dir,
    )
    server = create_http_server(args, node)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    host, port = server.server_address[:2]
    node.get_logger().info(f"mask designer UI: http://{host}:{port}/")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
