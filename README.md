# thesis-ns3

## Preparation
1. Install VirtualBox, and then install Ubuntu 16.04 on the VirtualBox. 
2. Install NS3 on Ubuntu and Li-Fi model on NS3.
> For the detailed procedure for the two steps above, please refer to the folder `NS3` of the lab server.

## Run Simulation

### Parameter Setting
We have the flexibility to configure various parameters by modifying the `global_configuration.h`.For instance, if we wish to set the total number of users to 30, we can simply assign the value 30 to the `UE_num` variable defined in `global_configuration.h`.

In addition, three macros `PROPOSED_METHOD` and `LASINR` and `LAEQOS` controls which LB method to be execute.

Besides, the macro `super_dynamic` indicates whether users can freely enter and exit rooms. If set to 1, the NS3 mobility model simulates user movement within the dimensions of the `room_size` in `global_configuration.h`. In this case, you can configure `room_size` to be larger than the **actual room size** and make modifications to the `check_indoor_user` function within the `proposed_method.cc` file.

### Batch Processing
The shell script `output.sh` is used to execute our simulation multiple times.

> shell script should be in the directory `/home/USER_NAME/repos/ns-3-allinone/ns-3.25`

This is an example of batch file. We can decide the number of times simulation should be conducted by changing the number `1000` to any value we want in line 3.

```shell=
#!/bin/bash

for i in {1,1000}; do
    ./waf --run scratch/thesis/thesis;
    date;
done
```


## Output file

Output file will be created in `/home/USER_NAME/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/ (or /benchmark/)`, depending on which LB method now is being executed.
If we do the simulation $N$ times, then there are $N$ rows in the output file, each of which has five values: *average user outage probability*, *average user satisfaction*, *average throughput (per user)*, *execution time* and *average jain's fairness index*.


:::warning
If you fail to run my code, you can check the following several things first:
1. Make sure that the value of the key `directory` of every object in `/home/USER_NAME/repos/ns-3.25/build/compile_commands.json` is equal to the path of `build` directory.
2. Make sure that the output path in `thesis.cc` is updated to match the machine you currently use.
3. If the error like "'>>' should be '> >' within a nested template argument list" appears, then cd to `/home/USER_NAME/repos/ns-3.25/` and enter the following command in terminal to explicitly specify the version of c++ compiler:
```shell=
CXXFLAGS="-Wno-error -std=c++11" ./waf configure
```
:::

#define PROPOSED_METHOD 0
#define LASINR 0
#define LAEQOS 1
#define SUPER_DYNAMIC 0