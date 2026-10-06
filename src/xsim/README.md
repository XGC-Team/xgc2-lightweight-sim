# xsim implementation

`core/` owns entity identity, components, commands and the single world roster.
`systems/` contains explicit robot preparation/steps and the sensor workers.
`models/` contains the original pure numerical headers and sealed PX4 sources.
`io/` contains config, native Unix RPC and optional ROS projection.
`main.cpp` composes the concrete world, systems and IO.

Configure `XSIM_ROS=OFF` for the headless native server and model/world/native
checks; `ON` enables the original ROS boundary. The build/install and public
contracts are in the [owning README](../../README.md).
