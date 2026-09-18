use jagua_rs::Instant;
use sparrow::config::{DEFAULT_SPARROW_CONFIG, TimeHalvingShrink};
use std::time::Duration;

#[test]
fn scheduled_shrink_uses_exact_boundaries_and_never_goes_below_floor() {
    let start = Instant::now();
    let mut config = DEFAULT_SPARROW_CONFIG.expl_cfg;
    config.shrink_step = 0.008;
    config.shrink_step_schedule = Some(TimeHalvingShrink {
        started_at: start,
        interval: Duration::from_secs(5),
        minimum_step: 0.001,
    });
    for (millis, expected) in [
        (0, 0.008),
        (4_999, 0.008),
        (5_000, 0.004),
        (9_999, 0.004),
        (10_000, 0.002),
        (14_999, 0.002),
        (15_000, 0.001),
        (20_000, 0.001),
        (86_400_000, 0.001),
    ] {
        assert_eq!(
            config.shrink_step_at(start + Duration::from_millis(millis)),
            expected
        );
    }
    assert_eq!(config.shrink_step_at(start - Duration::from_secs(1)), 0.008);
    // 重建调用复制配置时不能重新起算。
    let rebuilt = config;
    assert_eq!(
        rebuilt.shrink_step_at(start + Duration::from_secs(12)),
        0.002
    );
}

#[test]
fn default_and_disabled_schedule_keep_fixed_step() {
    let mut config = DEFAULT_SPARROW_CONFIG.expl_cfg;
    let now = Instant::now();
    assert!(config.shrink_step_schedule.is_none());
    assert_eq!(config.shrink_step_at(now + Duration::from_secs(100)), 0.001);
    config.shrink_step = 0.008;
    config.shrink_step_schedule = Some(TimeHalvingShrink {
        started_at: now,
        interval: Duration::ZERO,
        minimum_step: 0.001,
    });
    assert_eq!(config.shrink_step_at(now + Duration::from_secs(100)), 0.008);
}
