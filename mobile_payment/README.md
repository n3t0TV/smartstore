# Mobile Payment Package

## Installation steps

Clone the `setupcontainer` repository:
```
https://bitbucket.org/ErnestoTV/setupcontainer/src/master/
```

Enter the `setupcontainer` directory, and run the following script:
```
$ ./linux_config/idtech_sdk/install_idtech_sdk_and_deps.sh
```

Finally, compile the `mobile_payment` ROS package using catkin:
```
$ cd ~/catkin_ws
$ catkin_make
```

## How to run the payment reader node

Run the command:
```
$ roslaunch mobile_payment payment_reader.launch
```

## How to change the transaction amount, account, test mode?

Publish at the following ROS topics:
```
$ rostopic pub /payment_reader/test_transaction_mode std_msgs/Bool "data: false"
$ rostopic pub /payment_reader/transaction_amount std_msgs/Float32 "data: 0.0"
$ rostopic pub /payment_reader/transaction_timeout std_msgs/Int32 "data: 0"
$ rostopic pub /payment_reader/transaction_account mobile_payment_interface/WorldpayAccount "id: ''
token: ''
acceptor_id: ''
use_test_account: false"
```

## How to get transaction results?

Subscribe to the following ROS topic:
```
$ rostopic echo /payment_reader/worldpay_transaction_response
```

## How to pause/resume transactions?

Use the following ROS service:
```
$ rosservice call /payment_reader/transaction_enable "data: false"
```
