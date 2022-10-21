# README #

*Install files in a Matlab directory (Inverted_Pendulum_Cart)
* Run script pendulum_cart.m to define the system and calculate
* matricies CO and OB and the LQR gain vector without integral control.
* Uses full-state feedback.
* For integral control run pendulum_cart_integral.m.
* The Simulink file for integral control is Inv_pendulum_cart_aug_int.slx.

* I added pendulum_cart _integral.m to calulate the lqr integral control,
* At the same time this file has boolean flag 'cart' to model the Gantry Crane
* and boolean flag 'augmented' to add the integral control.

* Simulink file Inv_pendulum_cart_non_linear.slx is the full
* simulation using the non-linear plant model without integral control.
* Inv_pendulum_cart_plant is only the non-linear plant model for reference.

### What is this repository for? ###

* Matlab script examples for Inverted Pendulum on Cart using LQR
* This version is version 1.0.1

### How do I get set up? ###

* Uses Matlab R2022a

### License ###
* This software is distributed under the GNU General Public 
* Licence (version 3 or later); please refer to the file 
* Licence.md, included with the software, for details.
* Cite the source if republished or reditributed.

### Who do I talk to? ###
* Brian Tremaine
* brian@TremaineConsultingGroup.com
* 