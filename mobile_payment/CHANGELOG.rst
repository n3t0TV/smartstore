^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mobile-payment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2022-10-20 16:00:00 -0500)
---------------------------------
* Fixing fail time sleep, now when the transaction fails the reader will not
  sleep so that it can immediately accept another payment attempt.

1.2.0 (2022-09-19 17:00:00 -0500)
---------------------------------
* No changes in this package for this release.

1.1.0 (2022-09-01 17:00:00 -0500)
---------------------------------
* Monitor card taps in the payment reader after the reader is ready to receive
  payments. This was implemented by reading the STDOUT produced by the IDTech
  SDK because the SDK has no function or callback to achieve this.
* Publish card tap event in ROS and MQTT.
* Use the card tap event to play a "loading" audio in a loop until the
  transaction response arrives in the speaker node.
* Report failed transaction with ROS when the XML returned by WorldPay doesn't
  have a response.
* Publish failed transaction in ROS and MQTT when no XML response is published
  in the WorldPay callback.
* Append SKU and payment reader serial number in the reference number sent in
  WorldPay transaction request, and published in the iQ portal.
* Include time (hour, date, timezone), reference number and approval code in the
  WorldPay transaction message published in ROS and MQTT.
* Publish enable/disable status and test mode of the payment reader in ROS.

1.0.1 (2022-08-18 12:30:00 -0500)
---------------------------------
* Use the right paths for mss_utils include statements.

1.0.0 (2022-08-18 11:00:00 -0500)
---------------------------------
* Add first version of the Speaker node using mpg123 and only two audios for
  success or failure and a topic to change volume level.
* Use external timeout to restart the payment reader transaction in case the
  timeout of the reader is not triggered.
* Add configuration topic for the sleeping time after a successful transaction
  in payment reader.

0.1.0 (2022-08-10 09:20:00 -0500)
---------------------------------
* Update code to work with IDTech SDK v1.0.35.027
* Reduce maximum transaction timeout to 3 minutes, and set the 3 minutes as
  timeout default.
* Use the MSR_TIMEOUT callback to cancel the transaction.
* Set the Tortoise WorldPay account and disable test payment mode as default to
  process real payments.
* Respawn payment node if it crashes after 10 seconds.

0.0.7 (2022-07-22 14:10:00 -0500)
---------------------------------
* Move the payment_reader node launch file to the mobile-payment-interface
  package.

0.0.6 (2022-07-19 01:50:00 -0500)
---------------------------------
* Specify the mss_utils folder in the include header instructions for those
  headers coming from that package.

0.0.5 (2022-07-19 00:20:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.4 (2022-07-18 20:30:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.3 (2022-07-18 20:00:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.2 (2022-07-18 19:40:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.1 (2022-07-18 17:40:00 -0500)
---------------------------------
* First development code release.
