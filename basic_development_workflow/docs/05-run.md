## Run

In this section, we will run the application on a development x86 machine.

### Run with ESME

ESME is a daemon responsible for managing processes of the application. It uses a manifest file to configure and manage the processes.

1. Open a terminal and navigate to the build directory.

    ```
    cd build 
    ```

2. Locate the Manifest File

    An example manifest file is automatically generated in the build directory at `yaaac_codegen/deploy/noviroc/mw_1/esme`.

3. Execute the following command to run ESME with the manifest file.

    ```
    ./bin/esme yaaac_codegen/deploy/noviroc/mw_1/esme/esme_manifest_mw_1.bin -1
    ```

With the option -1, the application is started automatically.

Running application launched with ESME:

```
[       0.023607] |   INFO|       Producer|             producer.cpp@17   |       producer::producer       | sending number: 1
[       0.034759] |   INFO|       Consumer|             consumer.cpp@20   |       consumer::consumer       | received number: 1
[       0.034995] |   INFO|       Consumer|             consumer.cpp@27   |       consumer::consumer       | number is odd
[       0.223575] |   INFO|       Producer|             producer.cpp@17   |       producer::producer       | sending number: 2
[       0.234577] |   INFO|       Consumer|             consumer.cpp@20   |       consumer::consumer       | received number: 2
[       0.234680] |   INFO|       Consumer|             consumer.cpp@23   |       consumer::consumer       | number is even
...
[       3.223556] |   INFO|       Producer|             producer.cpp@17   |       producer::producer       | sending number: 17
[       3.234549] |   INFO|       Consumer|             consumer.cpp@20   |       consumer::consumer       | received number: 17
[       3.234659] |   INFO|       Consumer|             consumer.cpp@27   |       consumer::consumer       | number is odd
[       3.423404] |   INFO|       Producer|             producer.cpp@17   |       producer::producer       | sending number: 18
[       3.434314] |   INFO|       Consumer|             consumer.cpp@20   |       consumer::consumer       | received number: 18
[       3.434407] |   INFO|       Consumer|             consumer.cpp@23   |       consumer::consumer       | number is even
^C[       3.609753] |   INFO|           none|           executable.hpp@215  |       producer::none           | Terminate process because of a termination request.
[       3.649871] |   INFO|           none|           executable.hpp@215  |       consumer::none           | Terminate process because of a termination request.
```
ESME can be stoped by pressing `ctrl-c`in the terminal.

The Producer sends an incrementing number every 200 ms, the Consumer receives it and logs whether it is odd or even. Producer and Consumer run as independent processes with their own timestamps, so after pressing `ctrl-c` the exact interleaving and order of the two "Terminate process" log lines can vary between runs.


### Expected Outcome

Running the `esme` command above starts the Producer and Consumer processes for you, and you should see their logs interleaved in the terminal as shown above until you stop it with `ctrl-c`.

### Tip

Ensure that the build was successful before attempting to run the application.


Now that we have run the application, let's move on to recording data from the running application.

Previous section: [Visualize](04-visualize.md) | Next section: [Record](06-record.md)