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
$$
\theta =
\arcsin{
\begin{pmatrix}
{^{w}v_{E} \over \sqrt{^{w}v^{2}_{E}+^{w}v^{2}_{U}}}
\end{pmatrix}
}
$$