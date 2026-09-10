//! 满高挡板约束：禁止中间姿态跨过挡板的 X 区间，外框仍使用软碰撞。
use super::{ConstraintError, PlacementConstraints};
use jagua_rs::entities::{Instance, Item};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::spp::entities::SPInstance;

// 与源几何板归属转换的 f32 容差一致；不修改最终原始几何校验。
const BOUNDARY_EPS_MM: f32 = 0.001;

impl PlacementConstraints {
    /// 调用方显式把 Boundary 的整个 X 投影声明为禁止区域；不改变普通固定件语义。
    pub fn with_hard_x_barriers(mut self, instance: &SPInstance) -> Result<Self, ConstraintError> {
        self.validate_instance(instance)?;
        self.hard_x_barriers = self
            .placements()
            .filter(|f| self.is_boundary(f.item_id))
            .map(|f| {
                let bbox = instance
                    .item(f.item_id)
                    .shape_cd
                    .transform_clone(&f.pose.compose())
                    .bbox;
                (bbox.x_min, bbox.x_max)
            })
            .collect();
        self.hard_x_barriers.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(self)
    }

    pub fn has_hard_x_barriers(&self) -> bool {
        !self.hard_x_barriers.is_empty()
    }

    pub fn allows_bbox(&self, item_id: usize, bbox: Rect) -> bool {
        !self.has_hard_x_barriers()
            || self.is_fixed(item_id)
            || (bbox.x_min.is_finite()
                && bbox.x_max.is_finite()
                && self.hard_x_barriers.iter().all(|&(left, right)| {
                    bbox.x_max <= left + BOUNDARY_EPS_MM || bbox.x_min >= right - BOUNDARY_EPS_MM
                }))
    }

    pub fn allows_pose(&self, item: &Item, pose: DTransformation) -> bool {
        !self.has_hard_x_barriers()
            || self.is_fixed(item.id)
            || self.allows_bbox(item.id, item.shape_cd.transform_clone(&pose.compose()).bbox)
    }

    /// 把外部热启动转换产生的越挡板代理移到最近的合法 X 区域，旋转和 Y 保持不变。
    /// 零件间可能因此产生软碰撞，仍由分离器求解；不能作为物理可行证书。
    pub fn project_pose(&self, item: &Item, pose: DTransformation) -> Option<DTransformation> {
        if self.allows_pose(item, pose) {
            return Some(pose);
        }
        let bbox = item.shape_cd.transform_clone(&pose.compose()).bbox;
        let guard = 2. * f32::EPSILON * bbox.x_min.abs().max(bbox.x_max.abs()).max(1.);
        let mut best = None;
        let mut best_distance = f32::INFINITY;
        let mut left = f32::NEG_INFINITY;
        for right in self
            .hard_x_barriers
            .iter()
            .map(|v| v.0)
            .chain(std::iter::once(f32::INFINITY))
        {
            let lower = left + guard - bbox.x_min;
            let upper = right - guard - bbox.x_max;
            if lower <= upper {
                let delta = 0_f32.clamp(lower, upper);
                let (x, y) = pose.translation();
                let candidate = DTransformation::new(pose.rotation(), (x + delta, y));
                if delta.abs() < best_distance && self.allows_pose(item, candidate) {
                    best = Some(candidate);
                    best_distance = delta.abs();
                }
            }
            if let Some(wall) = self.hard_x_barriers.iter().find(|v| v.0 == right) {
                left = wall.1;
            }
        }
        best
    }

    /// 用容器截取挡板之间的区域，采样直接在可用区内选允许角度和位置。
    pub fn sample_regions(&self, container: Rect) -> Option<Vec<Rect>> {
        if !self.has_hard_x_barriers() {
            return None;
        }
        let mut regions = Vec::new();
        let mut left = container.x_min;
        for &(wall_left, wall_right) in &self.hard_x_barriers {
            if wall_right <= left {
                continue;
            }
            if let Ok(region) = Rect::try_new(
                left,
                container.y_min,
                wall_left.min(container.x_max),
                container.y_max,
            ) {
                regions.push(region);
            }
            left = left.max(wall_right);
            if left >= container.x_max {
                break;
            }
        }
        if let Ok(region) = Rect::try_new(left, container.y_min, container.x_max, container.y_max) {
            regions.push(region);
        }
        Some(regions)
    }
}
