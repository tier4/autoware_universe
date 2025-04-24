import torch
import torch.nn as nn

from autoware_naive_tf_planner.layers.embedding import PointsEncoder


class MapEncoder(nn.Module):
    def __init__(
        self,
        polygon_channel=4,
        dim=128,
    ) -> None:
        super().__init__()

        self.dim = dim
        self.polygon_encoder = PointsEncoder(polygon_channel, dim)
        self.route_mask_emb = nn.Embedding(2, dim)

        self.subtype_embed = nn.Embedding(16, dim)
        self.location_embed = nn.Embedding(4, dim)
        self.turn_direction_embed = nn.Embedding(3, dim)
        self.traffic_light_embed = nn.Embedding(5, dim) # 0~4 unknown, red, amber, green, white
        self.speed_limit_embed = nn.Sequential(
            nn.Linear(1, dim), nn.ReLU(), nn.Linear(dim, dim)
        )
        self.map_dropout = nn.Dropout1d(0.1)

    def forward(self, data) -> torch.Tensor:
        left_boundaries = data["boundary_left_boundaries"]  #[B, N_map, n_points, 2]
        right_boundaries = data["boundary_right_boundaries"] #[B, N_map, n_points, 2]
        boundary_masks = data["boundary_mask"] #[B, N_map]
        boundary_in_route = data["boundary_in_route"] # [B, N_map]
        subtypes = data["lanelet_subtypes"] #[B, N_map] long
        locations = data["lanelet_locations"] #[B, N_map] long
        turn_directions = data["lanelet_turn_directions"] #[B, N_map] long
        speed_limit = data["lanelet_speed_limit"] #[B, N_map] float
        traffic_light = data["lanelet_traffic_lights"] #[B, N_map] long

        boundary_features = torch.cat(
            [left_boundaries, right_boundaries], dim=-1
        ) #[B, N_map, n_points, 4]

        bs, M, P, C = boundary_features.shape
        boundary_masks = boundary_masks.view(bs * M, P)
        boundary_features = boundary_features.reshape(bs * M, P, C) #[B * N_map, n_points, base_dim]
        # print(boundary_features.shape)
        x_boundary = self.polygon_encoder(boundary_features, boundary_masks) 
        x_boundary = x_boundary.reshape(bs, M, self.dim) #[B, N_map, dim]

        x_boundary_route = self.route_mask_emb(boundary_in_route.long()) #[B, N_map, dim]
        x_subtype = self.subtype_embed(subtypes) #[B, N_map, dim]
        x_location = self.location_embed(locations) #[B, N_map, dim]
        x_turn_direction = self.turn_direction_embed(turn_directions) #[B, N_map, dim]
        x_speed_limit = self.speed_limit_embed(speed_limit.unsqueeze(-1)) #[B, N_map, dim]
        x_traffic_light = self.traffic_light_embed(traffic_light) #[B, N_map, dim]

        x_boundary = x_boundary + x_boundary_route + x_subtype + x_location + x_turn_direction + x_speed_limit + x_traffic_light
         
        x_boundary = self.map_dropout(x_boundary)
        return x_boundary