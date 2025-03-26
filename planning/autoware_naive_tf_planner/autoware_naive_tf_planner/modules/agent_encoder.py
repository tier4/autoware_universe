import torch
import torch.nn as nn

from autoware_naive_tf_planner.layers.common_layers import build_mlp
from autoware_naive_tf_planner.layers.embedding import NATSequenceEncoder


class AgentEncoder(nn.Module):
    def __init__(
        self,
        state_channel=6,
        history_channel=9,
        dim=128,
        hist_steps=21,
        use_ego_history=False,
        drop_path=0.2,
        state_attn_encoder=True,
        state_dropout=0.75,
    ) -> None:
        super().__init__()
        self.dim = dim
        self.state_channel = state_channel
        self.use_ego_history = use_ego_history
        self.hist_steps = hist_steps
        self.state_attn_encoder = state_attn_encoder


        self.obj_embedding = nn.Sequential(
            nn.Linear(4*4 + 3 + 4 * 2, dim),
            nn.LayerNorm(dim),
            nn.ReLU(),
            nn.Linear(dim, dim),
        )

        if self.use_ego_history:
            self.history_embedding = nn.Sequential(
                nn.Linear(10 * (16 + 3), dim),
                nn.LayerNorm(dim),
                nn.ReLU(),
                nn.Linear(dim, dim),
            )


        self.type_emb = nn.Embedding(8, dim) # 8 types of objects


    def forward(self, data):
        T = self.hist_steps

        position = data["objects_transform"]   #[B, N_obj, 4, 4]
        category = data["objects_types"].long() #[B, N_obj]
        velocity = data["objects_velocity"] #[B, N_obj, 3]
        shape = data["objects_footprint"] #[B, N_obj, 4, 2]
        valid_mask = data["objects_mask"] #[B, N_obj]

        B, N_obj = valid_mask.shape
        objs_feature  = torch.cat(
            [position.view(B, N_obj, -1), velocity, shape.view(B, N_obj, -1)], dim=-1
        ) #[B, N_obj, 4*4 + 3 + 4*2]

        objs_feature = self.obj_embedding(objs_feature) #[B, N_obj, dim]
        objs_feature = objs_feature * valid_mask.unsqueeze(-1) #[B, N_obj, dim]
        
        objs_type = self.type_emb(category) #[B, N_obj, dim]
        objs_feature = objs_feature + objs_type #[B, N_obj, dim]


        if self.use_ego_history:
            ego_pos_history = data["history_trajectories_transform"] # [B, N_h, 4, 4]
            ego_vel_history = data["history_trajectories_speed"] # [B, N_h, 3]
            B, N_h, _ = ego_pos_history.shape
            ego_pos_history = ego_pos_history.view(B, N_h, -1) #[B, N_h, 16]
            ego_vel_history = ego_vel_history.view(B, N_h, -1) #[B, N_h, 3]
            ego_history_feature = torch.cat(
                [ego_pos_history, ego_vel_history], dim=-1
            ) #[B, N_h, 19]
            ego_history_feature = self.history_encoder(
                ego_history_feature.view(B, 1, -1)
            ) #[B, 1, dim]

            objs_feature = torch.cat(
                [objs_feature, ego_history_feature], dim=1
            ) #[B, N_obj + 1, dim]
        
        return objs_feature
        
