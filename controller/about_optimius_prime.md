# Optimus Prime

## autobot!

__optimus_prime__ is functions that translate acc_command in NED to drone's coordinate in roll, pitch, yaw<br>
왜 optimus_prime 이냐?! .. 몰라요 묻지 마세요.. 내 맘이에요

## 1. _thust_to_RPY
This function translates acc_cmd in ENU coordinate to drone's body coordinate in roll, pitch, yaw, acc.<br>
Remember acc always towards z-direction of drone's coordinate.<br>
And coordinate ROTATES in 3 -> 2 -> 1 ORDER!!<br>

Please keep in mind these... and look at the image below..<br>
![optimus01](.././image/optimus01.png)

In ENU coordinate, drone's acc vector can be anywhere. But this vector must be aligned on drone body's z-direction.<br>
So, we can figure out drone's coordinate in roll, pitch, yaw by rotating to align coordinate's z-basis with the acc vector.<br>

First of all, we don't rotate yaw.. It's complicate.. I'll write this later..<br>

Second, rotate pitch. You can imagine triangle like OED in image above.<br>
In that triangle, $\angle EDO = {\pi \over 2}$ and $\angle EOD = \theta$. So, you can get $\theta$ easily with arcsin function. Like this,
$\theta = \arcsin {^{w}v_{E} \over \sqrt{^{w}v^{2}_{E} + ^{w}v^{2}_{U}}}$<br>

Third, rotate roll. In this time, you can imagine triangle like OEF in image above. It little complicate.. <br>
In that triangle, $\angle FEO = {\pi \over 2}$ and $\angle FOE = - \phi$. So, you can get $\phi$ easily as we did in Second. Like this,

$\phi = \arcsin {-^{w}v_{N} \over \lVert v \rVert}$<br>

This image is show what you did in this three step.
![optimus02](.././image/optimus02.png)

Yeah! We get roll, pitch, yaw, acc from aE, aN, aD command!

## 2. _thrust_to_ENU
This function translate command in drone's body coordinate to ENU coordinate.<br>

Our coordinate rotate in 321 order, so rotation matrix is consisted of each rotation matrix in 123 order like this formula.<br>
$R = R_{E}R_{N}R_{U}$<br>

Each rotation matrix can be derived from simple calculus.<br>
And we promised that we don't rotate yaw.<br>

Now we know rotation matrix that transform ENU coordinate vector to drone's body coordinate vector. Then, rotation matrix that transform drone's body coordinate vector to ENU coordinate vector is ... the transpose of rotation matrix! Because rotation matrix is orthonormal.<br>

![optimus03](.././image/optimus03.png)

So, acc vector in ENU coordinate is parallel with third column of transposed rotation matrix. With multiplying norm of acc, we can get aE, aN, aU from roll, pitch, yaw, acc(norm of acc).<br>

![optimus04](.././image/optimus04.png)

But you can realize something is strange.

필자도 아는데 어디서 이런 요상한 일이 일어났는지 모르겠으니 교수님한테 갈거임. 일단 이상한 점은 밑의 그림을 참조해주세요.

![optimus04](.././image/optimus05.png)
