^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2022-10-20 16:00:00 -0500)
---------------------------------
* Adding try-catch and timeout to services called from heartbeat node, this
  prevents provider service error from crash the node.
* Lock opening with sensor feedback instead of using delay-based event only,
  there is also a retries mechanism to make sure the container is opened.
* Lookup table adjustment and oversampling filter at battery measurements.
* Guard clause were added in battery driver when the I2C write function
  returns with a non zero value (-1), that means it was an error trying to
  communicate with the ADC, in that case the callback read function will return
  to avoid activating the low battery shutdown.

1.2.0 (2022-09-19 17:00:00 -0500)
---------------------------------
* Safe shutdown were added, when the battery reach the 5% level it will
  shutdown.
* MQTT remote shutdown were added, one instruction performs a reboot and the
  other poweroff the container.
* Current provider feedback added to heartbeat, now it will report any change
  about network provider (carrier).
* Fix applied to battery characterization in order to report a correct battery
  level.

1.1.0 (2022-09-01 17:00:00 -0500)
---------------------------------
* Implement switch between production and development environment in the MQTT
  manager, disconnecting from the old broker and then reconnect to the new one.
* Enable Version manager node to report the container version.
* Update battery characterization.
* Publish enable/disable status and test mode of the payment reader in the MQTT
  heartbeat.
* Enable/disable the test mode in the payment reader using the MQTT command
  "use_test_transactions" to use real cards or test cards.
* Rename the MQTT command "test" to "send_mock_transaction" to avoid ambiguity.

1.0.1 (2022-08-18 12:30:00 -0500)
---------------------------------
* No changes in this package for this release.

1.0.0 (2022-08-18 11:00:00 -0500)
---------------------------------
* Refactor MQTT Heartbeat node to merge the functionality of the Sensors Manager
  node and avoid having two different configurations for the rate of the
  heartbeat messages. In this refactor, the mss_utils library was used and now
  is a dependency of the container.
* Add toggle_provider instruction to the MQTT manager and apply google format
  to the MQTT manager source code.

0.1.0 (2022-08-10 09:20:00 -0500)
---------------------------------
* Convert the ADC lecture to percentage for the battery level.

0.0.7 (2022-07-22 14:10:00 -0500)
---------------------------------
* Remove legacy GPS code.
* Use the right GPIO pin number for the container lock.
* Fix the driver for the battery level reader.
* Rename the main launch file from test_mqtt to container.
* Add the payment_reader node in the container launch file.

0.0.6 (2022-07-19 01:50:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.5 (2022-07-19 00:20:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.4 (2022-07-18 20:30:00 -0500)
---------------------------------
* No changes in this package for this release.

0.0.3 (2022-07-18 20:00:00 -0500)
---------------------------------
* Remove backslashes from CMakeLists.txt as they actually introduce errors.

0.0.2 (2022-07-18 19:40:00 -0500)
---------------------------------
* Add missing backslash in CMakeLists.txt.

0.0.1 (2022-07-18 17:40:00 -0500)
---------------------------------
* First development code release.
