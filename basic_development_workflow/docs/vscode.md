## Visual Studio Code as IDE

The project ships with a `.vscode/` folder that pre-configures Visual Studio Code with C++ IntelliSense and debug configurations for this example.

### Using this example alongside an existing workspace

If you already have a VS Code workspace open (e.g. for the EAP or SDK installation), you don't need to switch to this folder as your workspace root. Instead, use **File → Add Folder to Workspace…** and, in the folder picker, select this `basic_development_workflow` folder itself — not a parent folder.

This is important: VS Code only looks for `.vscode/launch.json` directly at the top level of each folder you add. Since this file lives at `basic_development_workflow/.vscode/launch.json`, selecting a parent folder (e.g. the folder that contains multiple examples) means VS Code won't find it, and the debug configuration below won't show up at all.

VS Code resolves `${workspaceFolder}` in a folder's own `.vscode/launch.json` to that folder itself, regardless of which folder is primary in the workspace. This means the debug configuration below keeps working correctly once the folder is added — it'll simply appear in the Run and Debug dropdown alongside whatever configurations your other workspace folders already provide.

### Debugger Configurations (`launch.json`)

`launch.json` provides a GDB-based debug configuration accessible via the **Run and Debug** panel (`Ctrl+Shift+D`):

- **Attach to Recall Player** — attaches GDB to an already-running `dol_player_rest_server --recall-local` process (started from a terminal), so breakpoints in your business logic are hit during replay. Since `dol_player_rest_server` ships as a prebuilt tool rather than a project build artifact, its `program` path is resolved automatically: a `preLaunchTask` (see `tasks.json`) sources the run environment, looks up `dol_player_rest_server` via `command -v`, and links it into `.vscode/.dol_player_rest_server`, which `launch.json` points to.

### C++ IntelliSense (`c_cpp_properties.json`)

`c_cpp_properties.json` configures IntelliSense to resolve headers from the workspace and the Conan package cache:

```json
{
    "configurations": [{
        "name": "Linux",
        "includePath": [
            "${workspaceFolder}/**",
            "${env.HOME}/.conan/data/**",
            "/opt/**"
        ],
        "compilerPath": "/usr/bin/gcc",
        "cppStandard": "gnu++14",
        "intelliSenseMode": "gcc-x64"
    }]
}
```

