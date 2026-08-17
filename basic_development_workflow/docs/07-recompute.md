## Recompute

Recompute means replaying previously recorded data through your application and capturing the result again, without running the live system. It's useful for reproducing a run, comparing outputs, or investigating a bug offline. This section recomputes the Bytesoup file you recorded earlier and writes the result to a new Bytesoup file. If you also want to step through the business logic with a debugger while it recomputes, do this section first, then continue with **[Debug](08-debug.md)**.

For background on the underlying concept, see the
<a href="https://edms.etas.com/explanations/recompute.html" target="_blank">knowledge article on recompute</a>.

### Prerequisites

- The application must be built as described in the **[Build](03-build.md)** section.
- A Bytesoup file must be recorded as described in the **[Record](06-record.md)** section.

### Steps to recompute

1. Build for the recall target

    Rebuild the application with `MIDDLEWARE_TARGET=recall` to produce a version of your system, which the recall player can load. Run these commands from the root directory of the example. Skip this step if you already built for the recall target.

    >If you are working inside the EAP (Early Access Program) devcontainer instead of a standalone SDK installation, run this first:
    >```
    >export AOS_CHANNEL=eap
    >```

    ```bash
    conan install . -if=install -o MIDDLEWARE_TARGET=recall
    conan build . -if=install -bf=build
    ```

    This produces the system library `libDeployment_deploy_mw_1_recall_system.so` used by the recall player in step 3.

2. Source the run environment

    ```bash
    source ./install/activate_run.sh
    ```

3. Start the recall player

    `dol_player_rest_server` is a small local server that manages the recompute session; you control it with the `dol player` commands in the next steps. `--recall-local` makes it play the recording in this same process, instead of talking to a separate player process; `--system-library-path` tells it which application to recompute (the one you built in step 1); `--recorder-filepath` sets where it writes the replayed output. Starting it with `&` runs it in the background, so this terminal stays free for the commands that follow.

    ```bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/build/lib
    ```
    ```bash
    dol_player_rest_server \
        --recall-local \
        --recorder-filepath "recall_out.bs" \
        --system-library-path libDeployment_deploy_mw_1_recall_system.so &
    ```

4. Configure, open and play the recording

    ```bash
    dol player configuration --strategy recall_fixed
    dol player open recording.bs
    dol player play --attached
    ```

    `recall_fixed` assigns each runnable a fixed simulated runtime. `open` selects the recording to play back. `play` starts the recompute; `--attached` shows live progress until playback finishes or you press Ctrl+C.

5. Stop and quit

    ```bash
    dol player stop
    dol player quit
    ```

    `stop` ends the current playback; `quit` shuts down the player process started in step 3.
6. Inspect the result

    ```bash
    bytesoup_tools info recall_out.bs
    ```


### Tips

- Make sure the environment is sourced before starting.
- For timing-sensitive systems, it is possible to give runnables more realistic, individual runtimes instead of the `recall_fixed` default.

Congratulations! You have successfully recomputed your business logic into a new Bytesoup file using the recall player.

Previous section: [Record](06-record.md) | Next section: [Debug](08-debug.md) | [Back to beginning](../readme.md)
