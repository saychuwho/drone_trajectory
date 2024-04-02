# drone_trajectory

![img1](/image/thumbnail.png)

[webots](https://cyberbotics.com/)를 이용해 에너지 효율적인 drone의 경로를 생성하는 프로젝트이다. shortest-path finding algorithm 중 A* algorithm과 Theta* algorithm을 사용해서 제작되었다.

This project generates energy-efficient drone path based on [webots](https://cyberbotics.com/).

## Table

- [drone\_trajectory](#drone_trajectory)
  - [Table](#table)
  - [Pre-installation \& Execute](#pre-installation--execute)
  - [What's inside](#whats-inside)
    - [Environment representation](#environment-representation)
    - [shortest-path finding algorithms](#shortest-path-finding-algorithms)
    - [Energy Model for Drone](#energy-model-for-drone)
    - [Drone Controller](#drone-controller)
  - [References](#references)

## Pre-installation & Execute

1. [webots](https://cyberbotics.com/)을 설치한다.
2.  webots에서 사용하는 python에 `numpy`와 `matplotlib`를 설치한다. 만약 이미 `numpy`와 `matplotlib`가 설치된 python이 있거나 anaconda 환경이 있다면 해당 경로를 webots 설정에서 기본 python 실행 경로를 해당 python으로 설정해준다.
3.  그리고 webots에서 `worlds/drone_project.wbt`를 연 후 시뮬레이션을 시작하면 `controllers/grid_env_generator/data_YYYYMMDD` 폴더 밑에 생성된 path와 map의 plot이 저장되어 있다.
4.  만약 path와 map만 생성하고 싶다면, `numpy`와 `matplotlib`이 설치된 python을 이용해 [grid_env_generator_only_generation.py](/controllers/grid_env_generator/grid_env_generator_only_generation.py)를 실행하면 된다.

## What's inside

### Environment representation

[An efficient approach to near-optimal 3D trajectory design in cluttered environments for multirotor UAVs](https://doi.org/10.1109/COASE.2019.8842980)와 [Path Planning Strategies for UAVs in 3D Environments](https://doi.org/10.1007/s10846-011-9568-2)를 참고해 map, occupancy matrix, grid를 만든다

[map_generator.py](/controllers/grid_env_generator/map_generator.py)에 다음 함수들이 구현되어 map, matrix, occupancy matrix, grid를 생성한다.

- `map_gen()`: obstacle이 포함된 map을 생성한다
- `proto_option_gen()`: supervisor controller `grid_env_generator.py`가 webots world에 obstacle을 생성할 때 사용한다
- `matrix_gen()`: `map_gen()`을 이용해 생성한 map을 이용해 map matrix와 occupancy matrix를 만든다
- `grid_gen()`: occupancy matrix를 이용해 path 생성에 사용되는 grid를 만든다.
- `start_dest_generate()`: obstacle이 없는 빈 공간 중에서 시작 지점과 종료 지점을 지정한다

map은 x, y 각각 40의 너비를 가지고 있고, 높이는 10이다. 또한, 너비가 2x2, 4x4, 6x6이고 높이가 무작위인 obstacle 30개가 생성된다. 이 값들은 supervisor controller [grid_env_generator.py](/controllers/grid_env_generator/grid_env_generator.py)에 정의되어 있고, 이는 바꿀 수 있다. 다만, 너무 큰 사이즈의 map은 path를 생성하는데 많은 시간이 걸릴 수도 있다.

다음은 각각 생성된 map, occupancy matrix의 예시이다.

![map](/image/plot_20240308_map.png)

![occupancy matrix](/image/plot_20240308_occupancy.png)

### shortest-path finding algorithms

A* algorithm과 Theta* algorithm이 [path_generator.py](/controllers/grid_env_generator/path_generator.py)에 구현되어 있다.

알고리즘의 pseudo-code는 [Theta*:Any-Angle Path Planning on Grids](https://doi.org/10.1613/jair.2994)를 참고했다.

  - `a_star()`: Algorithm 1
  - `theta_star()`: Algorithm 3
  - `__post_smooth_path()`: Algorithm 2

`__lines_sight_partial_3D()`는 [An efficient approach to near-optimal 3D trajectory design in cluttered environments for multirotor UAVs](https://doi.org/10.1109/COASE.2019.8842980)의 [matlab code](https://github.com/danielesartori/3D-grid-path-planning/blob/master/line_sight_partial_3D.m)를 참고해 이를 python code로 변환했다.

생성된 path는 post-smoothing 과정을 거쳐서 보다 매끄러운 path로 바뀐다.

만들어진 path의 예시는 다음과 같다. 아래 path는 post-smoothing 과정을 거친 theta* path이다.

![theta path](/image/plot_path_20240308_smooth_theta_star.png)

다른 예시들은 여기서 확인할 수 있다. [A* path](/image/plot_path_20240308_a_star.png) / [A* path with post-smoothing](/image/plot_path_20240308_smooth_a_star.png) / [theta* path](/image/plot_path_20240308_theta_star.png)

### Energy Model for Drone

[energy_model.py](/controllers/drone_controller/energy_model.py)에 drone의 energy model을 구현했다.

Drone의 energy model은 [A power consumption model for multi-rotor small unmanned aircraft systems](https://doi.org/10.1109/ICUAS.2017.7991310)의 equation (14)와 equation (15)를 참고해 만들어졌다.

![](/image/table1_eq_14_eq_15.png)

Energy model에서 사용된 상수들은 [A power consumption model for multi-rotor small unmanned aircraft systems](https://doi.org/10.1109/ICUAS.2017.7991310)의 TABLE 1에 구하는 방법이 나와있지만, 이번 프로젝트에서는 TABLE 3의 상수들을 사용했다. 이는 energy model에 사용되는 다양한 상수들이 실험적으로 측정되어야 하지만, 현실 세계의 드론을 현재 가지고 있지는 않기 때문에 상수들을 구할 수 없었기 때문이다.

![](/image/table3.png)

에너지 모델은 shortest-path finding algorithm의 cost function에 사용된다. 구체적으로는 '각 점에서 목적지까지 소모되는 예상 에너지'를 `e_val`에 계산한 뒤, `a_star()`와 `theta_star()` 속 heap의 key 값으로 사용한다. 구체적인 cost function은 다음과 같다.

$$F(p) = k_gG(p) + k_hH(p) + k_eE(p)$$

$G(p)$는 `g_val`, $H(p)$는 `h_val`, $E(p)$는 `e_val`이다. $k_g, k_h, h_e$는 상수들로, 상수들은 실험적으로 조정되었다. 현재 상수값들은 확정되지 않았다.

### Drone Controller

[drone_controller.py](/controllers/drone_controller/drone_controller.py)를 이용해 webots world에 있는 drone을 움직인다. 현재는 수동 조작밖에 안되지만, 추후 생성된 path를 따라가도록 controller를 만들 것이다.

## References

1. D. Sartori, D. Zou and W. Yu, "An efficient approach to near-optimal 3D trajectory design in cluttered environments for multirotor UAVs," 2019 IEEE 15th International Conference on Automation Science and Engineering (CASE), Vancouver, BC, Canada, 2019, pp. 1016-1022, doi: https://doi.org/10.1109/COASE.2019.8842980.
2. De Filippis, L., Guglieri, G. & Quagliotti, F. Path Planning Strategies for UAVS in 3D Environments. J Intell Robot Syst 65, 247–264 (2012). https://doi.org/10.1007/s10846-011-9568-2
3. K. Daniel, A. Nash, S. Koeing, A. Felner, "Theta*:Any-Angle Path Planning on Grids," *Journal of Artifical Intelligence Research*, vol. 39, pp. 533-579, Oct 2010. https://doi.org/10.1613/jair.2994
4. Z. Liu, R. Sengupta and A. Kurzhanskiy, "A power consumption model for multi-rotor small unmanned aircraft systems," 2017 International Conference on Unmanned Aircraft Systems (ICUAS), Miami, FL, USA, 2017, pp. 310-315, doi: https://doi.org/10.1109/ICUAS.2017.7991310.