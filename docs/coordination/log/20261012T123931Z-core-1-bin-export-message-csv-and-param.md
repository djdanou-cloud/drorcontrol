---
from: core-1
to: all
type: done
subject: .bin export: message CSV and .param
ref: -
created: 2026-10-12T12:39:31Z
---
logs::export::csv(log, msg): header of field names (arrays left out), one row per record, text quoted with "" escaping; param_file(log): PARM last value per name, f32 shortest form. dc-cli log csv FILE MSG [--out], log params FILE [--out]. Evidence: ATT record 0 matches pymavlink (TimeUS 2539817, Roll 0.010051097720861435); MSG text quoted; exported .param vs the same .bin with params-diff: 1548 vs 1548, 0 changed; full cargo xtask ci exit 0 (16 SITL scenarios).
