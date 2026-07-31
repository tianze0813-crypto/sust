# Debug Session: ui-not-opening
- **Status**: [OPEN]
- **Issue**: `python main.py` 后浏览器无法打开前端界面
- **Debug Server**: Pending
- **Log File**: `.dbg/trae-debug-log-ui-not-opening.ndjson`

## Reproduction Steps
1. Activate the `sustechpoints` conda environment.
2. Run `python main.py` in `SUSTechPOINTS`.
3. Open `http://127.0.0.1:8081` in a browser.

## Hypotheses & Verification
| ID | Hypothesis | Likelihood | Effort | Evidence |
|----|------------|------------|--------|----------|
| A | CherryPy service never reaches listen state on port 8081 | High | Low | Pending |
| B | Import-time model load or missing dependency causes process exit before server starts | High | Low | Pending |
| C | Browser is opening the wrong host or the home page request is not reaching the service | Medium | Low | Pending |
| D | Service starts but static/template resources fail, causing blank or broken page | Medium | Medium | Pending |

## Log Evidence
- User terminal evidence:
  - `conda activate sustechpoints` fails with `CommandNotFoundException`.
  - `python main.py` fails at import time with `ModuleNotFoundError: No module named 'cherrypy'`.
  - Port 8081 is not listening when checked from IDE terminal.
- Subsequent user evidence in Anaconda environment:
  - TensorFlow model loads successfully and warmup inference completes.
  - CherryPy reaches `ENGINE Serving on http://0.0.0.0:8081` and `ENGINE Bus STARTED`.
  - CherryPy emits checker warnings for `auth.require`, `./temp`, `./views`, and `./assets`, but these are warnings rather than startup blockers.

## Verification Conclusion
- Hypothesis A: Confirmed for the earlier wrong shell, but rejected for the Anaconda shell. In the correct environment, the service does reach listen state on 8081.
- Hypothesis B: Confirmed as the root cause of earlier failures only. It does not explain the latest run in Anaconda.
- Hypothesis C: Now the most likely explanation. Browser access method or address is likely wrong while the service is already running.
- Hypothesis D: Still possible, but lower priority because the homepage should respond as soon as the CherryPy service is up.
