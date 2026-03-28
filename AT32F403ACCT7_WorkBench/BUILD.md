# Build Notes

## Keil Build

- Project file: `E:\Li_FOC\AT32F403ACCT7_WorkBench\project\MDK_V5\AT32F403ACCT7_WorkBench.uvprojx`
- Working directory: `E:\Li_FOC\AT32F403ACCT7_WorkBench\project\MDK_V5`
- Keil executable: `E:\tools\keil5\UV4\UV4.exe`
- Expected toolchain for this project: `ARM Compiler 5 (AC5)`

## Script Entry

```powershell
.\build-script.ps1
```

Extensionless entry:

```powershell
.\build
```

This resolves to `build.cmd`, which launches `build-script.ps1` with `-ExecutionPolicy Bypass`.

Force close UV4 before building:

```powershell
.\build-script.ps1 -ForceCloseUV4
```

Or:

```powershell
.\build -ForceCloseUV4
```

By default, the script will build even if `UV4` is already running, so you can keep Keil open for debugging.

Batch entry:

```powershell
.\build.bat
```

Direct Keil command:

```powershell
& 'E:\tools\keil5\UV4\UV4.exe' -b 'E:\Li_FOC\AT32F403ACCT7_WorkBench\project\MDK_V5\AT32F403ACCT7_WorkBench.uvprojx' -j0
```

## VSCode Task

- Default build task: `Keil Build`
- Optional task: `Keil Build (Force Close UV4)`

## Flash Script

Flash the current output with Keil:

```powershell
.\flash
```

Build first, then flash:

```powershell
.\flash -BuildFirst
```

Force close UV4 before building/flashing:

```powershell
.\flash -BuildFirst -ForceCloseUV4
```

## Output

- Build log: `E:\Li_FOC\AT32F403ACCT7_WorkBench\project\MDK_V5\objects\AT32F403ACCT7_WorkBench.build_log.htm`
- Target output: `E:\Li_FOC\AT32F403ACCT7_WorkBench\project\MDK_V5\objects\AT32F403ACCT7_WorkBench.axf`
- Hex output: `E:\Li_FOC\AT32F403ACCT7_WorkBench\project\MDK_V5\objects\AT32F403ACCT7_WorkBench.hex`

## Notes For Future Sessions

- Use the workspace scripts or VSCode tasks directly; no custom status bar button is required.
- Do not modify source code just to adapt toolchain differences unless explicitly requested.
- This repo has previously failed under AC6 because the included FreeRTOS RVDS port expects AC5 syntax.
- If build fails, check the HTML build log first for the exact file and line number.
