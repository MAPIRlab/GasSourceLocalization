# Usage

You can run any specific simulation by doing:

```
ros2 launch pmfs_env gaden_preproc_launch.py scenario:=<scenario> simulation:=<simulation>
```

```
ros2 launch pmfs_env gaden_sim_launch.py scenario:=<scenario> simulation:=<simulation>
```

```
ros2 launch pmfs_env main_simbot_launch.py scenario:=<scenario> simulation:=<simulation> method:=<method>
```

The parameters for the simulation and the GSL algorithms can be modified under `scenarios/<scenario>/params` and `scenarios/<scenario>/simulations`.

# Scenarios

The following wimage shows the configuration used for each experiment. The source position is marked with a square, the starting robot position is marked with a circle, and the airflow inlets and outlets are marked with arrows.
![Image of all the experiment scenarios](resources/all_configs.png)



## Starting distance and optimal navigation time

Distance form starting position to source location, along with time necessary to travel it assuming 0.4 m/s constant speed.


| Exp  | Distance |   Time    |
|------|----------|-----------|
|  A1  |  5.71 m  |  14.29 s  |
|  A2  |  10.55 m |  26.38 s  |
|  A3  |  7.24 m  |  18.10 s  |
|  A4  |  4.87 m  |  12.17 s  |
|  B1  |  9.21 m  |  23.01 s  |
|  B2  |  7.76 m  |  19.39 s  |
|  B3  |  8.49 m  |  21.21 s  |
|  B4  |  5.43 m  |  13.58 s  |
|  C1  |  7.10 m  |  17.76 s  |
|  C2  |  5.62 m  |  14.06 s  |
|  C3  |  7.76 m  |  19.39 s  |
|  C4  |  8.65 m  |  21.63 s  |
|  D1  |  6.08 m  |  15.21 s  |
|  D2  |  5.53 m  |  13.83 s  |
|  D3  |  2.77 m  |  6.92 s   |
|  D4  |  4.84 m  |  12.10 s  |
|  E1  |  5.91 m  |  14.77 s  |
|  E2  |  6.21 m  |  15.54 s  |
|  E3  |  8.32 m  |  20.79 s  |
|  E4  |  6.07 m  |  15.17 s  |