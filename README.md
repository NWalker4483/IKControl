# IkControl : Flexible Inverse Kinematic Solution for Hobby 6DOFs

` Depends on AxisControl lib`

## Usage

### 1. Create Your robot control object inhereting from the MultiAxis<6> class described in [AxisControl](https://github.com/NWalker4483/AxisControl)

```cpp
#include <AxisControl.h>

class MyRobot : public MultiAxis<6> {

}
```

### 2. Create an IkController Object, define DH parameters and attach the robot to the controller

```cpp
#include <IKControl.h>
#include <MyRobot.h>

MyRobot robot;
double DHparams[6][4] = { ... }
IkController controller;

void setup(){
    controller.attach(robot, robot.run);
    controller.setDHParams(DHparams);
}
```

### 3. Run Moves

```cpp
void loop(){
    if (controller.canExecute()){
    controller.move( ... ); // TODO: Consider Combining move and jog calls 
    controller.jog( ... );
    }
}
```

## Task 1

- [ ] Rewriting the [AR4 Firmware code](examples/ar4_main.ino) using the [AxisControl](https://github.com/NWalker4483/AxisControl) Library
