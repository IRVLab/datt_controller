# Diver Attention Controller for Aqua AUV
* Reads the predicted diver attention values from the `diver_attention` node
* Publishes control command for Aqua AUV to "navigate-and-reorient" for initiating an interaction

## How to RUN
* 2 terminals required

## (1) - Vision -- DATT Controller Node
* Run: `roslaunch datt_controller datt_controller_aqua.launch`

## (2) - Vision -- BAG Record
* Run: `roslaunch datt_controller datt_controller_record.launch my_args:="name_of_the_exp"` (eg, "attentive1", "not_attentive_left1", "not_attentive_right1", etc.)

## Summary
* If the subject is attentive, the robot will perform a slight pitch down and up to indicate that it got the attention of the diver.
* If the subject is NOT attentive, the robot will try to grab its attention by slightly rolling right and left. And, then the robot will "navigate-and-reorient" itselft with respect to the subject.

## MODIFICATION for EXPERIMENTS (to achieve a better "navigate-and-reorient" phase)
* The modifications need to be done in the following script: `datt_controller/src/rc_aqua.py`
* To tune the parameters so we can achieve a better "navigate-and-reorient" phase, we need to go to the following functions `def right_manuever(self, pred_topic):` and `def left_manuever(self, pred_topic):`. The variables to change are 
```
vx = 0.25
pseudo_dist = abs(pred_topic.mask)//3 # <--- if you use a larger number, the robot thinks that the subject is closer, and vice-versa
```
    * A higher `pseudo_dist` means the robot will make a bigger semi-circle during the "navigate-and-reorient" phase
* Currently, we are making a prediciton once we see at least `3` consistent predictions. You may have to increase or decrease this number (very unlikely). This can be changed here `if self.attentive_meter.count > 3:`, `elif self.right_not_attentive_meter.count > 3:`, and `elif self.left_not_attentive_meter.count > 3:`.

## Issues
* `/aqua_base` doesn't show up and hence, the controller can not start. The controller code was fine when it was last run on September 9, 2022 in pool. Both Michael and Junaed saw this error before an outreach event in our lab, but we never found a fix. Because of this, none of the rcvm or rrcomm code work. My best guess is that the catkin_make within aqua_base_ws (whoever did that, if anyone, after Sept 9 trial) was not done properly but I am not sure about this.
