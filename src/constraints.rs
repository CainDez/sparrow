//! 固定姿态属于副本约束；Boundary 仅供调用方构造内部边界，不能当作生产件导出。
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
use jagua_rs::entities::{Instance, Layout};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixedRole {
    Production,
    Boundary,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FixedPlacement {
    pub item_id: usize,
    pub pose: DTransformation,
    pub role: FixedRole,
}
impl FixedPlacement {
    pub fn production(item_id: usize, pose: DTransformation) -> Self {
        Self {
            item_id,
            pose,
            role: FixedRole::Production,
        }
    }
    pub fn boundary(item_id: usize, pose: DTransformation) -> Self {
        Self {
            item_id,
            pose,
            role: FixedRole::Boundary,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConstraintError {
    UnknownItem(usize),
    DuplicateItem(usize),
    AmbiguousQuantity(usize),
    NonFinitePose(usize),
    ForbiddenRotation(usize),
    MissingOrChangedPose(usize),
    FixedOutside(usize),
    FixedCollision(usize),
}
impl std::fmt::Display for ConstraintError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "固定姿态约束无效：{self:?}")
    }
}
impl std::error::Error for ConstraintError {}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlacementConstraints {
    fixed: BTreeMap<usize, FixedPlacement>,
}
impl PlacementConstraints {
    pub fn new(instance: &SPInstance, fixed: Vec<FixedPlacement>) -> Result<Self, ConstraintError> {
        let mut map = BTreeMap::new();
        for f in fixed {
            let id = f.item_id;
            let Some((item, qty)) = instance.items.get(id).filter(|(item, _)| item.id == id) else {
                return Err(ConstraintError::UnknownItem(id));
            };
            if *qty != 1 {
                return Err(ConstraintError::AmbiguousQuantity(id));
            }
            let (x, y) = f.pose.translation();
            if !x.is_finite() || !y.is_finite() || !f.pose.rotation().is_finite() {
                return Err(ConstraintError::NonFinitePose(id));
            }
            let allowed =
                crate::sample::uniform_sampler::convert_sample_to_closest_feasible(f.pose, item);
            if jagua_rs::geometry::normalize_rotation(allowed.rotation() - f.pose.rotation()).abs()
                > 1e-5
            {
                return Err(ConstraintError::ForbiddenRotation(id));
            }
            if map.insert(id, f).is_some() {
                return Err(ConstraintError::DuplicateItem(id));
            }
        }
        Ok(Self { fixed: map })
    }
    pub fn is_empty(&self) -> bool {
        self.fixed.is_empty()
    }
    pub fn is_fixed(&self, id: usize) -> bool {
        self.fixed.contains_key(&id)
    }
    pub fn is_boundary(&self, id: usize) -> bool {
        self.fixed
            .get(&id)
            .is_some_and(|f| f.role == FixedRole::Boundary)
    }
    pub fn placements(&self) -> impl Iterator<Item = &FixedPlacement> {
        self.fixed.values()
    }
    pub fn validate_instance(&self, instance: &SPInstance) -> Result<(), ConstraintError> {
        Self::new(instance, self.fixed.values().cloned().collect()).map(|_| ())
    }

    pub fn initialize(&self, prob: &mut SPProblem) -> Result<(), ConstraintError> {
        self.validate_instance(&prob.instance)?;
        for f in self.fixed.values() {
            if prob
                .layout
                .placed_items
                .values()
                .any(|pi| pi.item_id == f.item_id)
            {
                return Err(ConstraintError::DuplicateItem(f.item_id));
            }
            prob.place_item(SPPlacement {
                item_id: f.item_id,
                d_transf: f.pose,
            });
        }
        let min_width = self.minimum_width(prob);
        if min_width > prob.strip_width() {
            prob.change_strip_width(min_width);
        }
        self.validate_layout(&prob.layout)
    }
    pub fn matches_solution(&self, solution: &SPSolution) -> bool {
        self.fixed.values().all(|f| {
            let mut poses = solution
                .layout_snapshot
                .placed_items
                .values()
                .filter(|pi| pi.item_id == f.item_id);
            poses.next().is_some_and(|pi| pi.d_transf == f.pose) && poses.next().is_none()
        })
    }
    pub fn validate_layout(&self, layout: &Layout) -> Result<(), ConstraintError> {
        for f in self.fixed.values() {
            let mut items = layout
                .placed_items
                .iter()
                .filter(|(_, pi)| pi.item_id == f.item_id);
            let Some((pk, pi)) = items.next() else {
                return Err(ConstraintError::MissingOrChangedPose(f.item_id));
            };
            if pi.d_transf != f.pose || items.next().is_some() {
                return Err(ConstraintError::MissingOrChangedPose(f.item_id));
            }
            let mut hits = BasicHazardCollector::new();
            layout.cde().collect_poly_collisions(&pi.shape, &mut hits);
            hits.remove_by_entity(&HazardEntity::from((pk, pi)));
            for (_, hazard) in hits.iter() {
                match hazard {
                    HazardEntity::Exterior if !self.is_boundary(f.item_id) => {
                        return Err(ConstraintError::FixedOutside(f.item_id));
                    }
                    HazardEntity::PlacedItem { pk: other, .. }
                        if self.is_fixed(layout.placed_items[*other].item_id) =>
                    {
                        return Err(ConstraintError::FixedCollision(f.item_id));
                    }
                    _ => (),
                }
            }
        }
        Ok(())
    }
    pub fn minimum_width(&self, prob: &SPProblem) -> f32 {
        self.fixed
            .values()
            .map(|f| {
                let item = prob.instance.item(f.item_id);
                let x = item.shape_cd.transform_clone(&f.pose.compose()).bbox.x_max;
                if f.role == FixedRole::Boundary {
                    x
                } else {
                    x + prob.strip.shape_modify_config.offset.unwrap_or(0.) + 1e-5 * x.abs().max(1.)
                }
            })
            .fold(0., f32::max)
    }
    pub fn fit_initial_strip(&self, prob: &mut SPProblem) {
        if self.is_empty() {
            prob.fit_strip();
            return;
        }
        let real_end = prob
            .layout
            .placed_items
            .values()
            .filter(|pi| !self.is_boundary(pi.item_id))
            .map(|pi| pi.shape.bbox.x_max)
            .fold(0., f32::max);
        let width = (real_end * 1.00001 + prob.strip.shape_modify_config.offset.unwrap_or(0.))
            .max(self.minimum_width(prob))
            .max(1e-4);
        prob.change_strip_width(width);
    }
}
