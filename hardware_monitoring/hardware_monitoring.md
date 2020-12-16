# Monitor the hardware load

The Hardware Monitor is needed to collect all relevant information about the hardware system and to monitor and document the hardware performance during the simulation run.

While [hardware_monitor.py](hardware_monitoring\hardware_monitor.py) is able to gather detailed hardware data based on a python library, its overhead has a negative effect on the simulation speed. For this reason, we only use it to document the hardware setup information before start of every simulation run.

To document the hardware load during the simulation run, we use the C-based [collectl](http://collectl.sourceforge.net/) package on the start of a benchmark run in [run_all_benchmarks.sh](run_all_benchmarks.sh). It is a lightweight hardware monitor running on a seperated thread. It saves the data in two different text formats. [hardware_data_postprocessor.py](hardware_monitoring\hardware_data_postprocessor.py) extracts all relevant informations from the output file after each benchmark run. [hardware_data_assigner.py](hardware_monitoring\hardware_data_assigner.py) assigns the data, based on the timestamp, to the corresponding data points in the JSON result files. This way all data of a simulation run is stored in one file.

## Install

Python based Hardware Monitor:

```shell
pip install psutil GPUtil
```

```shell
sudo apt install collectl
```

## Collectl Commands

Start recording:

```shell
collectl --all -i 0.1 -f $path &
```

Decompress .gz file in verbose mode:

```shell
monitor_file=$(find $path -type f -name '*.gz')
echo "Converting hardware monitor file: $monitor_file"

collectl -sCmD -o Tm -p $monitor_file >> ${monitor_file::-3}
```

Decompress .gz file in brief mode:

```shell
monitor_file=$(find $path -type f -name '*.gz')
echo "Converting hardware monitor file: $monitor_file"

collectl -scmd -o Tm -p $monitor_file >> ${monitor_file::-3}
```

### Ouput formats (CPU and Memory)

Brief mode:

```shell
#             <----CPU[HYPER]-----><-----------Memory----------->
#Time         cpu sys inter  ctxsw Free Buff Cach Inac Slab  Map
15:47:24.251    2   0  2804   4008 693M 199M   1G 958M 370M   4G
15:47:24.501    1   1  2452   3716 693M 199M   1G 958M 370M   4G
15:47:24.751    8   1  3564   7292 693M 199M   1G 958M 370M   4G
15:47:25.001    2   0  2676   4208 693M 199M   1G 958M 370M   4G
15:47:25.251   11   8  3372   4948 694M 199M   1G 958M 370M   4G
15:47:25.501    3   0  3084   5336 694M 199M   1G 958M 370M   4G
15:47:25.751    7   1  2928   5584 694M 199M   1G 958M 370M   4G
15:47:26.001    3   1  3004   5528 694M 199M   1G 958M 370M   4G
15:47:26.251    1   0  2740   4608 694M 199M   1G 958M 370M   4G
15:47:26.501    2   0  2720   4308 694M 199M   1G 958M 370M   4G
15:47:26.751    3   0  2776   4540 694M 199M   1G 958M 370M   4G
15:47:27.001    1   0  2620   4004 694M 199M   1G 958M 370M   4G
15:47:27.251    1   0  2488   3792 694M 199M   1G 958M 370M   4G
15:47:27.501    1   0  2380   3552 694M 199M   1G 958M 370M   4G
15:47:27.751    2   1  2972   4340 694M 199M   1G 958M 370M   4G
```

Verbose mode:

```shell
### RECORD    1 >>> PC-Name <<< (1600781272.001) (Tue Sep 22 15:27:52.001 2020) ###

# SINGLE CPU[HYPER] STATISTICS
#   Cpu  User Nice  Sys Wait IRQ  Soft Steal Guest NiceG Idle
      0     4    0    0    0    0    0     0     0     0   95
      1     3    0    0    0    0    0     0     0     0   96
      2     4    0    0    0    0    0     0     0     0   96
      3    30    0    0    0    0    0     0     0     0   69
      4     0    0    0    0    0    0     0     0     0  100
      5     0    0    0    0    0    0     0     0     0  100
      6     8    0    0    0    0    0     0     0     0   92
      7     0    0    0    0    0    0     0     0     0  100

# DISK STATISTICS (/sec)
#          <---------reads---------------><---------writes--------------><--------averages--------> Pct
#Name       KBytes Merged  IOs Size  Wait  KBytes Merged  IOs Size  Wait  RWSize  QLen  Wait SvcTim Util
sda              0      0    0    0     0       0      0    0    0     0       0     0     0      0    0

# MEMORY SUMMARY
#<------------------------------------Physical Memory------------------------------------------><-----------Swap------------><-------Paging------>
#   Total    Used    Free    Buff  Cached    Slab  Mapped    Anon   AnonH  Commit  Locked Inact Total  Used  Free   In  Out Fault MajFt   In  Out
    7847M   7671M 180620K 119208K   1989M 326232K   1000M   5055M       0  17554M     80K  617M 2047M  203M 1844M    0    0  1836     0    0    0

### RECORD    2 >>> PC-Name <<< (1600781272.251) (Tue Sep 22 15:27:52.251 2020) ###

# SINGLE CPU[HYPER] STATISTICS
#   Cpu  User Nice  Sys Wait IRQ  Soft Steal Guest NiceG Idle
      0     3    0    3    0    0    0     0     0     0   92
      1     0    0    4    0    0    0     0     0     0   96
      2     0    0    0    0    0    0     0     0     0  100
      3    11    0    0    0    0    0     0     0     0   88
      4     3    0    0    0    0    0     0     0     0   96
      5     0    0    0    0    0    0     0     0     0  100
      6     4    0    0    0    0    0     0     0     0   96
      7     7    0    0    0    0    0     0     0     0   92

# DISK STATISTICS (/sec)
#          <---------reads---------------><---------writes--------------><--------averages--------> Pct
#Name       KBytes Merged  IOs Size  Wait  KBytes Merged  IOs Size  Wait  RWSize  QLen  Wait SvcTim Util
sda              0      0    0    0     0       0      0    0    0     0       0     0     0      0    0

# MEMORY SUMMARY
#<------------------------------------Physical Memory------------------------------------------><-----------Swap------------><-------Paging------>
#   Total    Used    Free    Buff  Cached    Slab  Mapped    Anon   AnonH  Commit  Locked Inact Total  Used  Free   In  Out Fault MajFt   In  Out
    7847M   7671M 180984K 119208K   1989M 326240K    999M   5054M       0  17554M     80K  617M 2047M  203M 1844M    0    0   416     0    0    0
```

## Links

Examples:
<http://collectl.sourceforge.net/Examples.html>

Detailed data:
<http://collectl.sourceforge.net/FAQ-collectl>

CPU time -> CPU load:
<https://serverfault.com/questions/648704/how-are-cpu-time-and-cpu-usage-the-same>
