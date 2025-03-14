&nbsp;
<div align=center>
  <img src="https://github.com/Technician13/PCQR/blob/master/img/LOGO/PCQR.svg?sanitize=true" width="300px">
</div>

<div align="center">
<h1>An Integrated Motion Planning and Control<br /> Framework for Quadruped Robot Based on<br /> Trajectory Optimization and MPC</h1>
    <a href='https://orcid.org/0000-0002-1577-4573' target='_blank'>Donglin Chen</a>;
    <a href='https://orcid.org/0009-0004-6947-368X' target='_blank'>Dehong Cong</a>;
    <a href='https://orcid.org/0009-0003-4224-9234' target='_blank'>Jingchao Qiao</a>;
    <a href='https://orcid.org/0009-0005-0218-5955' target='_blank'>Ming Zhao</a>;
    <a href='https://orcid.org/0000-0001-7279-731X' target='_blank'>Xiangman Song</a>;
    <a href='https://orcid.org/0000-0002-9950-5169' target='_blank'>Lixin Tang</a>;
</div>

<div align="center">
<h3><strong>The paper is submitted to <a href='https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=6221036' target='_blank'>TCYB 2024</a></strong></h3>
</div>

<div align="center">    
  
[![Video Badge](https://img.shields.io/badge/-video-009E2F)](https://www.youtube.com/)
![Static Badge](https://img.shields.io/badge/Ubuntu-passing-2C448B)     

</div>

## Introduction
In this work, we propose an integrated framework, designated PCQR (motion planning and control for quadruped robots). This framework consists of two components for motion planning and for motion control. We propose whole-body planning and state estimation algorithms in the former to together achieve motion planning for all degrees of freedom of the quadruped robot. In the whole-body planning algorithm, we divide a humongous motion planning task into three subtasks for foothold selection, swing leg trajectory generation, and body trajectory generation to reduce computational complexity and improve the real-time performance of motion planning. We implement a two-stage controller in the latter, including a main controller based on model predictive control and single-leg controllers to compute joint torque commands. Experimental results confirm the effectiveness of PCQR, as well as its superiority in locomotion stability, terrain adaptability, and real-time performance compared to some popular baselines.
<div align="center">
    <img src="img/FigWorkflow.png", width="800">
</div>


## Setup
Major dependency: 
* [Ipopt 3.12.7](https://www.coin-or.org/download/source/Ipopt/)
* [OSQP 0.4.1](https://osqp.org/)
* [RBDL](https://rbdl.github.io/)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [Webots 2021a](https://www.cyberbotics.com/doc/blog/Webots-2021-a-release)

Download   
```
git clone https://github.com/Technician13/PF-GBP-python
```


## Running
You can run the project with the default configuration in the `controllers/Actiger_controller/commonFunction.hpp`. 
```
mkdir build
cd build
cmake ..
make -j
```
Set up the controller in Webots and run it. The remote control is used to control robot behavior.
<div align="center">
    <img src="img/FigRC.png", width="800">
</div>


## Citation 
If you find this repository is useful, please consider citing our paper:




