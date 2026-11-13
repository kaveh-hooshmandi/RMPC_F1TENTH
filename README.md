# RMPC F1TENTH: simulation and reproducible evaluation

**Maintainer and paper author:** Kaveh Hooshmandi, Arak University of Technology, [k.hooshmandi@arakut.ac.ir](mailto:k.hooshmandi@arakut.ac.ir).



![Simplified F1-style circuit and recorded lateral response](paper/figures/local_f1_track_response.png)

The circuit is an original design inspired by Yas Marina, not official track geometry. The lateral value in the plot is signed **Frenet offset from the centerline**, not global map y.

## Run everything from one terminal

With the project Docker image already built, open a desktop terminal in this repository:

```bash
./run.sh launch f1
```

This ROS 2 launch starts the F1TENTH simulator, GPU RMPC controller, map server, Foxglove bridge, and RViz. Press **Ctrl+C** to stop the launched processes. The Docker service stays available; use `./run.sh down` when finished.

```bash
./run.sh launch f1 --no-rviz  # omit RViz
./run.sh launch f1 --record   # also save odometry and drive to a ROS bag
./run.sh launch clear         # original IMS track
./run.sh launch obstacles     # IMS with obstacles
```

The `--record` option saves a new UTC-stamped directory under `telemetry/`. `launch/all_in_one.launch.py` combines the ROS components. The commands were tested on this workstation with an RTX 3050; a short recorded run produced a valid bag with both requested topics.

## Prepare a new machine

The tested setup uses Linux with an X11 desktop, Docker Engine and Compose, an NVIDIA GPU with its host driver and NVIDIA Container Toolkit, and the `osrf/ros:humble-desktop-full` base image. ROS Humble and CUDA 12.4 are installed **inside** the project image. Building needs network access to fetch pinned simulator revisions and packages. The Dockerfile currently targets CUDA compute capability **8.6** (RTX 3050); change `CMAKE_CUDA_ARCHITECTURES=86` for another GPU before building.

The build installs dependencies inside a new container image. Obtain any approval required for installation on your machine before running it. Skip the build if `jamris-rmpc-f1tenth:local` is already present.

```bash
./run.sh check
./run.sh build --install-approved
./run.sh launch f1
```

If the ROS base image is absent, fetch it through your approved Docker setup first. `./run.sh check` reports image and GPU availability. After starting the service, `./run.sh verify` checks CUDA, ROS packages, and Python imports.

`compose.local.example.yaml` shows optional `/dev/dri` mappings for this workstation. Copy it to `compose.local.yaml` only if your display needs them. The local override is ignored by Git. The default Compose file uses an NVIDIA GPU reservation without fixed `/dev/dri` device numbers.

## راهنمای فارسی: نصب Docker و اجرای پروژه

این راهنما برای **اوبونتو 22.04 یا 24.04 با محیط گرافیکی X11 و کارت NVIDIA** است. درایور NVIDIA باید روی میزبان نصب باشد و دستور `nvidia-smi` کارت را نشان دهد. ROS 2 و CUDA هنگام ساخت تصویر پروژه داخل Docker نصب می‌شوند؛ نیازی به نصب آن‌ها روی میزبان نیست. برای نصب درایور متناسب با کارت و نسخهٔ اوبونتو، [راهنمای رسمی NVIDIA](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/ubuntu.html) را ببینید.

### ۱. نصب Docker Engine و Compose

اگر Docker و افزونهٔ Compose از قبل نصب و قابل استفاده‌اند، این مرحله را رد کنید. در غیر این صورت، مخزن رسمی Docker را اضافه کنید و بسته‌ها را نصب کنید ([مستندات رسمی Docker](https://docs.docker.com/engine/install/ubuntu/)):

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo docker run --rm hello-world
```

اسکریپت `run.sh` دستورهای Docker را بدون `sudo` اجرا می‌کند. برای دادن این دسترسی به حساب خود، دستور زیر را بزنید و **یک بار از نشست کاربری خارج و دوباره وارد شوید**. عضویت در گروه `docker` دسترسی در سطح مدیر سیستم می‌دهد؛ [توضیح Docker](https://docs.docker.com/engine/install/linux-postinstall/) را بخوانید.

```bash
sudo usermod -aG docker "$USER"
```

پس از ورود دوباره، `docker version` و `docker compose version` باید بدون `sudo` کار کنند.

### ۲. نصب NVIDIA Container Toolkit

پس از اطمینان از کارکرد `nvidia-smi` روی میزبان، مخزن NVIDIA را اضافه و runtime مربوط به Docker را تنظیم کنید ([مستندات رسمی NVIDIA](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)):

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### ۳. ساخت و اجرای پروژه

در **ترمینال محیط گرافیکی X11** وارد پوشهٔ `RMPC_F1TENTH` شوید؛ همان پوشه‌ای که `run.sh` در آن است. `run.sh` به متغیر `DISPLAY` و فایل قابل‌خواندن `XAUTHORITY` (معمولاً `~/.Xauthority`) نیاز دارد. ابتدا تصویر پایه را دریافت کنید. سپس تصویر پروژه را بسازید و اجرا کنید:

```bash
docker pull osrf/ros:humble-desktop-full
./run.sh check
./run.sh build --install-approved
./run.sh up
./run.sh verify
./run.sh launch f1
```

ساخت تصویر به اینترنت نیاز دارد و ممکن است زمان‌بر باشد. دستور `build --install-approved` وابستگی‌ها را **داخل تصویر Docker** نصب می‌کند. معماری CUDA در `docker/Dockerfile` روی `86` برای RTX 3050 تنظیم شده است؛ اگر کارت دیگری دارید، پیش از ساخت مقدار `CMAKE_CUDA_ARCHITECTURES` را متناسب با آن تغییر دهید. پس از اجرای `launch`، با **Ctrl+C** برنامه‌ها را متوقف کنید و سپس سرویس Docker پروژه را ببندید:

```bash
./run.sh down
```

برای اجرای بدون RViz از `./run.sh launch f1 --no-rviz` استفاده کنید. اگر نمایش RViz مشکل داشت، توضیح `compose.local.example.yaml` در بخش انگلیسی بالا را ببینید.

## Recorded run and figures

The archived F1-style run is `telemetry/run_20261112_115749/`. Two laps were detected: 63.696 s and 59.412 s; their mean is 61.554 s. Its recorded maximum absolute centerline offset is 0.662 m. This is **one asynchronous ROS run**, not a repeated-trial comparison. [The run report](reports/f1-style-run-summary.md) explains its scope.

After `./run.sh launch f1 --record` and Ctrl+C, use the new bag directory name:

```bash
./run.sh analyze BAG_DIRECTORY f1
./run.sh plot-f1 BAG_DIRECTORY
```

`analyze` writes metrics and general plots to `results/`. `plot-f1` writes the track and lateral-offset figure. `scripts/generate_f1_style_track.py` generates the original circuit; the map and centerline are under `src/rmpc_f1tenth/maps/F1_STYLE/`.

## Controlled comparison

The separate synchronous study evaluates Pure Pursuit and RMPC at 1,000, 2,500, and 5,000 samples on IMS and the new circuit, plus a synthetic low-grip sector. Its 100 archived trial traces are in `experiments/study_v1/` (80 nominal) and `experiments/slippery_v1/` (20 stress). See [the protocol](experiments/README.md) and [the fixed report](experiments/report/summary.md).

To run new trials after starting the container, choose fresh output directory names:

```bash
./run.sh down
./run.sh up
./run.sh benchmark-build
./run.sh benchmark nominal my_nominal_repeat
./run.sh benchmark slippery my_slippery_repeat
./run.sh report
```

`benchmark-build` compiles source already present in the image; it does not install packages. The report command currently reads archived `study_v1` and `slippery_v1`; edit its dataset paths to report newly named repeats. The study's timing covers synchronized solver calls, not ROS scheduling deadlines.
