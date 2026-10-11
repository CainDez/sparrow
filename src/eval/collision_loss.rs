use crate::quantify::quantify_collision_poly_container;
#[cfg(not(feature = "simd"))]
use crate::quantify::quantify_collision_poly_poly;
#[cfg(feature = "simd")]
use crate::quantify::simd::circles_soa::CirclesSoA;
#[cfg(feature = "simd")]
use crate::quantify::simd::quantify_collision_poly_poly_simd_bounded;
use crate::quantify::tracker::CollisionTracker;
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::primitives::SPolygon;

/// Computes Sparrow's collision loss as `jagua-rs` discovers hazards.
pub(super) struct CollisionLossEvaluator<'a> {
    layout: &'a Layout,
    ct: &'a CollisionTracker,
    current_pk: PItemKey,
    loss: f32,
    loss_bound: f32,
    #[cfg(feature = "simd")]
    poles_soa: CirclesSoA,
}

impl<'a> CollisionLossEvaluator<'a> {
    pub(super) fn new(layout: &'a Layout, ct: &'a CollisionTracker, current_pk: PItemKey) -> Self {
        Self {
            layout,
            ct,
            current_pk,
            loss: 0.0,
            loss_bound: f32::INFINITY,
            #[cfg(feature = "simd")]
            poles_soa: CirclesSoA::new(),
        }
    }

    pub(super) fn reload(&mut self, loss_bound: f32, _shape: &SPolygon) {
        self.loss = 0.0;
        self.loss_bound = loss_bound;
        #[cfg(feature = "simd")]
        self.poles_soa.load(&_shape.surrogate().poles);
    }

    pub(super) fn add(&mut self, hazard: HazardEntity, shape: &SPolygon) -> bool {
        // 上游 #161：无限预算下即使累积损失溢出，也不得产生 Infinity - Infinity。
        let remaining = if self.loss_bound == f32::INFINITY {
            f32::INFINITY
        } else {
            self.loss_bound - self.loss
        };
        let Some(extra_loss) = self.calc_weighted_loss_bounded(hazard, shape, remaining) else {
            return true;
        };
        self.loss += extra_loss;
        self.loss > self.loss_bound
    }

    pub(super) fn loss(&self) -> f32 {
        self.loss
    }

    fn calc_weighted_loss_bounded(
        &self,
        hazard: HazardEntity,
        shape: &SPolygon,
        max_loss: f32,
    ) -> Option<f32> {
        match hazard {
            HazardEntity::PlacedItem { pk: other_pk, .. } => {
                let other_shape = &self.layout.placed_items[other_pk].shape;
                let weight = self.ct.get_pair_weight(self.current_pk, other_pk);

                #[cfg(feature = "simd")]
                {
                    quantify_collision_poly_poly_simd_bounded(
                        other_shape,
                        shape,
                        &self.poles_soa,
                        max_loss / weight,
                    )
                    .map(|loss| loss * weight)
                }

                #[cfg(not(feature = "simd"))]
                {
                    let loss = quantify_collision_poly_poly(other_shape, shape) * weight;
                    (loss <= max_loss).then_some(loss)
                }
            }
            HazardEntity::Exterior => Some(
                quantify_collision_poly_container(shape, self.layout.container.outer_cd.bbox)
                    * self.ct.get_container_weight(self.current_pk),
            ),
            _ => unimplemented!("unsupported hazard entity"),
        }
    }
}

#[cfg(test)]
mod upstream_regressions {
    use super::*;
    use crate::config::DEFAULT_SPARROW_CONFIG;
    use jagua_rs::entities::Item;
    use jagua_rs::geometry::geo_enums::RotationRange;
    use jagua_rs::geometry::primitives::Rect;
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::geometry::{DTransformation, OriginalShape};
    use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, Strip};

    fn problem() -> SPProblem {
        let item = Item::new(
            0,
            OriginalShape {
                shape: SPolygon::from(Rect::try_new(0., 0., 10., 10.).unwrap()),
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            },
            RotationRange::None,
            None,
            DEFAULT_SPARROW_CONFIG.cde_config.item_surrogate_config,
        )
        .unwrap();
        let strip = Strip::new(
            100.,
            DEFAULT_SPARROW_CONFIG.cde_config,
            ShapeModifyConfig::default(),
            100.,
        )
        .unwrap();
        let mut p = SPProblem::new(SPInstance::new(vec![(item, 2)], strip));
        p.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0., (20., 20.)),
        });
        p.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0., (40., 20.)),
        });
        p
    }

    #[test]
    fn unbounded_loss_overflow_keeps_following_collisions_evaluable() {
        let p = problem();
        let tracker = CollisionTracker::new(&p.layout);
        let keys = p.layout.placed_items.keys().collect::<Vec<_>>();
        let other = &p.layout.placed_items[keys[1]];
        let mut evaluator = CollisionLossEvaluator::new(&p.layout, &tracker, keys[0]);
        evaluator.reload(f32::INFINITY, &other.shape);
        // 模拟先前的外框损失/权重溢出；后续碰撞仍必须完成无上限评估。
        evaluator.loss = f32::INFINITY;
        assert!(!evaluator.add(HazardEntity::from((keys[1], other)), &other.shape));
        assert_eq!(evaluator.loss(), f32::INFINITY);
    }

    #[test]
    fn finite_budget_still_rejects_loss_after_overflow() {
        let p = problem();
        let tracker = CollisionTracker::new(&p.layout);
        let keys = p.layout.placed_items.keys().collect::<Vec<_>>();
        let other = &p.layout.placed_items[keys[1]];
        let mut evaluator = CollisionLossEvaluator::new(&p.layout, &tracker, keys[0]);
        evaluator.reload(1., &other.shape);
        evaluator.loss = f32::INFINITY;
        assert!(evaluator.add(HazardEntity::from((keys[1], other)), &other.shape));
    }
}
