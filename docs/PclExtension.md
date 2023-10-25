# RGL PCL extension

The extension introduces the nodes and functions for point cloud processing. RGL features are enhanced with the following:

- Downsampling
- Writing to PCD file
- Visualization

For operations listed above, the extension uses [Point Cloud Library](https://pointclouds.org/).

## Building

Before building RGL PCL extension, it is necessary to install the required dependencies. Run the setup script with the `--install-pcl-deps` flag to download and install them. It could take some time (Point Cloud Library is built from source):

```bash
# Linux:
./setup.py --install-pcl-deps
# Windows:
python setup.py --install-pcl-deps
```

To build RGL with PCL extension, run the setup script with `--with-pcl`:

```bash
# Linux:
./setup.py --with-pcl
# Windows:
python setup.py --with-pcl
```

## API documentation

More details can be found [here](../include/rgl/api/extensions/pcl.h).
