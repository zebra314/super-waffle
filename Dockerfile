FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------- #
#                               Environment setup                              #
# ---------------------------------------------------------------------------- #

# Specify terminal color
ENV TERM=xterm-256color

# Install basic tools
RUN apt-get update && \
    apt-get install -y neovim && \
    apt-get install -y zsh && \
    apt-get install -y tmux && \
    apt-get install -y curl && \
    apt-get install -y git && \
    apt-get install -y build-essential

# Setup zsh
RUN chsh -s $(which zsh)

# Install oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Install p10k
RUN git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
RUN sed -i 's/ZSH_THEME="robbyrussel}l"/ZSH_THEME="powerlevel10k\/powerlevel10k"/g' ~/.zshrc
COPY dotfiles/.p10k.zsh /root/.p10k.zsh
COPY dotfiles/.zshrc /root/.zshrc

# Install multiple zsh plugins
# 1. zsh-autosuggestions
# 2. zsh-syntax-highlighting
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
RUN sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/g' ~/.zshrc

# Allow to run GUI
RUN apt-get update && \
    apt-get install -y libpci-dev && \
    apt-get install -y x11-apps && \
    apt-get install -y qtwayland5 && \
    apt-get install -yqq xserver-xorg && \
    apt-get install -y xwayland && \
    apt-get install -y qt5-default

# ---------------------------------------------------------------------------- #
#                             ROS environment setup                            #
# ---------------------------------------------------------------------------- #

# Install ROS packages
RUN apt-get update
RUN apt-get install -y ros-noetic-rviz
RUN apt-get install -y ros-noetic-gazebo-ros-control
RUN apt-get install -y ros-noetic-gazebo-ros-pkgs
RUN apt-get install -y ros-noetic-xacro 
RUN apt-get install -y ros-noetic-amcl
RUN apt-get install -y libopencv-dev
RUN apt-get install -y ros-noetic-map-server
RUN apt-get install -y ros-noetic-moveit
RUN apt-get install -y ros-noetic-costmap-2d
RUN apt-get install -y ros-noetic-four-wheel-steering-msgs

# # Install ROS dependencies using rosdep
# # Take approximately 15 minutes
# RUN apt-get install -y python3-rosdep
# RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
#         rosdep init; \
#     fi && \
#     rosdep update
# COPY ./tiago_ws /root/tiago_ws
# RUN rosdep install --from-paths /root/tiago_ws/src --ignore-src -r -y

# Set workspace
WORKDIR /root/taigo_ws

# Entry point
COPY scripts/entrypoint.sh /root/scripts/entrypoint.sh
RUN chmod +x /root/scripts/entrypoint.sh
ENTRYPOINT ["/root/scripts/entrypoint.sh"]
# CMD ["sh", "-c", "clear && exec zsh"]
CMD ["sh", "-c", "exec zsh"]