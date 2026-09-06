# Elexhub synthetic comparison

Internal simulation validation with provisional dimensions and actuator parameters. C uses oracle TCP feedback.

Every rejection remains a failed trial. Error quantiles cover completed trials only.

|Condition|Success / attempted|Wilson 95% interval|Median completed error (mm)|Aborted|
|---|---:|---:|---:|---:|
|A|100 / 100|0.963–1.000|5.16|0|
|B|100 / 100|0.963–1.000|1.80|0|
|C|100 / 100|0.963–1.000|1.80|0|

A: nominal servo model. B: provisional gravity compensation. C: B plus bounded oracle TCP correction.

These are software checks, not validation of the Elexhub hardware or an RGB feedback experiment.

All three conditions reached the 15 mm success ceiling. B and C produced identical endpoint errors across the paired goals; this run demonstrates no incremental C benefit. Paired success bootstrap intervals degenerate at this ceiling and do not establish equivalence. The frozen goals, threshold and recorded failures were not changed after observing results.
