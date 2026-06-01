# Debug Session: cpu-crash [OPEN]

## Symptom
- Backend process exits unexpectedly even when TensorFlow GPU is disabled.
- Prior symptom also included silent return to PowerShell without Python traceback.

## Scope
- Project: `SUSTechPOINTS`
- Environment: Windows, conda env `sustechpoints`

## Falsifiable Hypotheses
1. CherryPy autoreloader spawns/monitors a child process and the observed "crash" is actually the worker process exiting or being restarted unexpectedly.
2. A non-GPU native dependency still crashes the Python process on CPU path, such as TensorFlow CPU runtime, NumPy MKL/OpenMP, or another compiled extension.
3. The `/checkscene` or another heavy request triggers memory pressure or a native-level fault during large-response generation, causing abrupt process termination.
4. The process is being terminated externally by OS/security tooling rather than raising a Python exception, which would explain the lack of traceback and normal shutdown logs.
5. The current startup path or environment activation differs between runs, causing inconsistent interpreter/DLL resolution and unstable runtime behavior.

## Evidence To Collect
- Exact exit code after process returns to shell.
- Whether the crash still happens with CherryPy autoreloader disabled.
- Windows Event Viewer / WER application error entry around crash time.
- Last successful request before exit and whether `/checkscene` is a reliable trigger.
- Imported module path and interpreter path used by the crashing process.

## Next Step
- Collect runtime evidence without changing business logic.

## Evidence Collected
- Reproduced in external terminal and in CPU-only mode.
- Process exit code: `-1073741819` which maps to `0xC0000005` (Access Violation).
- Windows Event Viewer `Application Error`:
  - Faulting application: `D:\miniconda3\envs\sustechpoints\python.exe`
  - Faulting module: `nvdxgdmal64.dll`
  - Exception codes observed: `0xc0000005` and `0xc000041d`
- Windows Error Reporting entries:
  - Event names: `APPCRASH`, `BEX64`
  - Faulting module reported as `nvdxgdmal64.dll_unloaded`

## Hypothesis Status
1. CherryPy autoreloader is the primary root cause.
   - Status: weakened, not confirmed.
   - Reason: crash signature is native access violation in `nvdxgdmal64.dll`, which is stronger evidence than a Python/cherrypy-level restart issue.
2. A non-GPU native dependency still crashes the Python process on CPU path.
   - Status: partially confirmed.
   - Reason: CPU-only still crashes with native exit code; however the named faulting module is still an NVIDIA graphics DLL rather than TensorFlow CPU runtime itself.
3. A heavy request like `/checkscene` triggers the crash.
   - Status: plausible but unproven.
   - Reason: prior crash happened after requests, but current evidence does not isolate a single endpoint as the trigger.
4. Process is terminated externally or by native module, not by Python exception.
   - Status: confirmed.
   - Reason: no traceback, no clean CherryPy shutdown, WER/Application Error show native crash.
5. Interpreter/DLL resolution instability contributes to runtime failure.
   - Status: plausible.
   - Reason: crashing module is an unloaded NVIDIA DLL injected into the Python process, suggesting environment/driver/display-stack interaction.

## Current Best Conclusion
- This is a native Windows crash, not a normal Python exception.
- The strongest evidence points to interaction with `nvdxgdmal64.dll` (NVIDIA display/graphics stack) even when the app is started in CPU-only mode.
- Therefore, simply disabling TensorFlow GPU is not sufficient; the crash likely involves graphics-driver interaction with the Python process, browser/WebGL/display path, or another native library loaded into the process.
