# PandaNest 上游兼容回迁

本 fork 以 `f92e74deae57448086f4b7de198f5f3bbb8320eb` 为定制基线；共同上游祖先为
`b881c98d4373ebd372eac114980944df3b6a065e`。本批评估冻结上游 main
`4629b22795ae6b25b3b460ccde6fb36a71292b6d`，采用兼容回迁，不能视为已合并完整 main。
保留 MIT 许可证及源文件署名。

| 上游来源 | 迁入资产 | 目标契约适配 |
| --- | --- | --- |
| `ce4a382` / #161 | 无限碰撞预算溢出修复 | 保留提前终止收集器、固定件和挡板约束；有限预算行为保持原样 |
| `9ef4567` / #159 | 有界初解构造、typed error、单点采样区间 | 逐件和搜索内取消仍受同一预算控制；启发式失败返回 `None`，不声明数学不可行；保留合法 warm best 的上层路径 |
| `ed1c72c` / #162 | 面积与离散旋转的必要宽度下界 | 排除人工 Boundary 面积；保留容器 inset、固定件跨度和硬边界；连续旋转仅用面积界 |
| `7f0e10f` / #165 | 对齐 Jagua 0.8.3 版本声明 | 由父 workspace patch 固定到带 PandaNest 保守 offset 补偿的 Jagua fork |

本 fork crate 为 `0.2.0`，仍使用 Jagua 0.8 API。`LBFBuilder::construct` 和
`construct_with_listener` 返回 `Result`；调用者必须处理 `ConstructionError`。
可中断构造仍可能返回部分状态，上层可行性证书负责拒绝不完整候选。

新增测试先在旧实现上复现无限预算、精确贴边和不可构造初解失败，再验证修复。
下界另有固定旋转和人工挡板测试。真实数据、取消、固定姿态、跨板移动和收缩调度测试保留。

上游 1.x 类型、反射输入、collector/filter、CLI、记录布局和性能日志重构未迁入。
PandaNest 不使用 Python wrapper；产品算法版本、两份 lockfile 与完整对照报告由父仓库管理。
