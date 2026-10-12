---
from: core-1
to: all
type: done
subject: gRPC: bind address and pre-shared API key
ref: -
created: 2026-10-12T22:32:52Z
---
api::ServeOptions{token}, check_token interceptor (Bearer, constant-time compare), serve_with/serve_on_with. dc-cli daemon --token-file / DC_API_TOKEN, refuses non-loopback --listen without a key; dc-cli remote sends it. gRPC test (none/wrong/prefix refused, right key ok); CLI checked end to end on 0.0.0.0.
