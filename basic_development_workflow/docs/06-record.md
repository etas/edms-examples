## Record

In this section, we will record data from the running application on the development machine. This is useful for debugging during the development phase, especially when the application is not running on a real target.

### Prerequisites

- The option `LOGGING_FRAMEWORK` in the `conanfile.py` must be set to `console` (this is already the default, so no change is needed unless you switched it to `no_console`).

### Steps to record data

1. Build the application for x86 target

    Ensure the application is built for the x86 target as described in the **[Build](03-build.md)** section.

2. Run the application

    Start the application as described in the **[Run](05-run.md)** section, and keep that terminal running for the duration of the recording.

3. Start the MTA RAW gateway

    Open a new terminal in the project root directory (the application from step 2 must keep running in its own terminal) and source the environment:

    ```bash
    source ./install/activate_run.sh
    ```

    Register the gateway as its own process within the middleware, then start recording. `ACTIVITY_INSTANCE_NAME` can be any name of your choosing; `PROCESS_ID` must follow the pattern `ProcessId_<ACTIVITY_INSTANCE_NAME>_Gateway`:

    ```bash
    export ACTIVITY_INSTANCE_NAME=raw_gateway
    export PROCESS_ID=ProcessId_raw_gateway_Gateway
    mta_raw_output_gateway --enable-noviroc-manifest -f recording.bs
    ```

    > `--enable-noviroc-manifest` tells the gateway to use the ESME noviroc manifest to discover the sender ports to record. `-f recording.bs` sets the output file.

    Recording with MTA RAW Gateway:

    ```
    Required library interface version: '1.0.0'
    Found shared library interface version: '1.0.0'
    Required AOS base interface version: '1.1.0'
    Found shared library AOS base interface version: '1.1.0'
    mta_gateway_generic v0.1.0 commit: workspace
    Transport Library: mta_gateway_ostream v0.11.0 commit: workspace
    Logging setup done for level 
    ```

    > No "Dropped data" warnings should appear here. The Producer activity is cyclically triggered every 200 ms (`cycle_ms: 200` in `activities/Producer.activity.yaml`), matching the `raw_gateway` measurement config's own poll cycle (`cycle_ms: 200` in `deployment/Deployment.deploy.yaml`), so the gateway's 2-slot cache (`default_cache_size: 2`) never gets overwritten between polls. If you see "Dropped data" warnings, check that these two `cycle_ms` values are still aligned — a Producer sending faster than the gateway polls will cause samples to be genuinely lost from the recording.

4. Stop recording

    Once you have recorded enough data, stop the recording by pressing Ctrl+C.


### Inspect the recorded file with bytesoup tools

>**Note:** If you reuse the terminal used for MTA recording, you don't need to source the environment. Otherwise you have to source `./install/activate_run.sh` before.

1. Index the recorded data

    ```
    bytesoup_tools index recording.bs
    ```

    > This generates a `recording.bs.idx` file, which allows tools to access samples ordered by timestamp and filtered by time range instead of only reading them sequentially. It also enables tools such as the recall player (see **[Recompute](07-recompute.md)**) to open the recording.

2. Get information about the recorded data (optional)

    ```
    bytesoup_tools info recording.bs
    ```

    > This prints metadata about the recording (duration, number of messages, contained types and services) so you can sanity-check that the expected data was actually captured before moving on to analysis.

Using bytesoup_tools for getting information about the recorded data:

```
path:                   recording.bs
size:                   4.9K
version:                0.1.0.0
abi flags:              11082838
compression:            None
duration:               00:00:03 (hh:mm:ss)
first sample timestamp: 13680416400313703ns
last sample timestamp:  13680419400320349ns
messages:               16
number of class infos:  2
types:                  consumer_producer_example::Message
                        example::InternalState
services:               Message.producer__output.Message                                         16 msgs  :  consumer_producer_example::Message
                        Producer_StateService.producer/producer__State.ProducerState_StateEvent   0 msgs  :  example::InternalState
```

### Expected Outcome

- Recording: The recording.bs file should be created with the recorded data.
- Inspection: You should be able to index and get information about the recorded data using Bytesoup tools.

### Tips

- Ensure that the application is running correctly before starting the recording.

Now that we have recorded the data, let's move on to analyze or system based on recompute.

Previous section: [Run](05-run.md) | Next section: [Recompute](07-recompute.md)