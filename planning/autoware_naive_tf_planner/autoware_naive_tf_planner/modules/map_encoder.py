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
        self.route_mask_emb = nn.Sequential(
            nn.Linear(1, dim), nn.ReLU(), nn.Linear(dim, dim)
        )

    def forward(self, data) -> torch.Tensor:
        left_boundaries = data["boundary_left_boundaries"]  #[B, N_map, n_points, 2]
        right_boundaries = data["boundary_right_boundaries"] #[B, N_map, n_points, 2]
        boundary_masks = data["boundary_mask"] #[B, N_map]
        boundary_in_route = data["boundary_in_route"] # [B, N_map]
        boundary_features = torch.cat(
            [left_boundaries, right_boundaries], dim=-1
        ) #[B, N_map, n_points, 4]

        bs, M, P, C = boundary_features.shape
        boundary_masks = boundary_masks.view(bs * M, P)
        boundary_features = boundary_features.reshape(bs * M, P, C) #[B * N_map, n_points, base_dim]
        # print(boundary_features.shape)
        x_boundary = self.polygon_encoder(boundary_features, boundary_masks) 
        x_boundary = x_boundary.reshape(bs, M, self.dim) #[B, N_map, dim]

        x_boundary_route = self.route_mask_emb(boundary_in_route.float().unsqueeze(-1)) #[B, N_map, dim]
        x_boundary = x_boundary + x_boundary_route

        return x_boundary
