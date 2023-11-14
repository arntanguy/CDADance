# CDADance

This controller publishes the controller's code used during the live demo held at Centre des Arts in September 2023.

<!-- [![demo video](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2019-17-45.png)](https://youtu.be/XoXDZBgbamc) -->

## How to install

### Pre-requisites

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [lipm_walking_controller](https://github.com/jrl-umi3218/lipm_walking_controller)
- [mc_xsens_plugin](https://github.com/arntanguy/mc_xsens_plugin)
- [hrp4_description](https://github.com/isri-aist/hrp4_description)
- [mc-hrp4](https://github.com/isri-aist/mc-hrp4)
- [ismpc_walking]() [optional]

### Build and install

```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -8
sudo make install
```

### How to run

In your `mc_rtc.yaml` configuration file:

```yaml
MainRobot: HRP4::CDA
Enabled: CDADance
Timestep: 0.005
```

Then you can run in your favorite dynamic simulator (choreonoid, mujoco, ...)

## Thanks

- Arnaud TANGUY for developping this controller and related tools
- Hui-Ting Hong for developping this controller and related tools
- Rocio Berenguer
- Centre des Arts
- Eurartec
- LIRMM - IDH Team
