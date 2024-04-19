ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG RECIPES_BRANCH=master
ARG RECIPES_PROVIDER=https://github.com/ADVRHumanoids/multidof_recipes.git

ARG FOREST_WS=$HOME/modular_ws
ARG USER_PWD=user
ARG MODE=&NoNe&
ARG JOBS=1

# run apt update as root
USER root
SHELL ["/bin/bash", "-ic"]
RUN sudo apt-get update

USER user

# Add github.com to known hosts
RUN mkdir -p -m 0700 $HOME/.ssh && ssh-keyscan github.com >> $HOME/.ssh/known_hosts

# Initialize forest workspace
WORKDIR ${FOREST_WS}
RUN forest init
RUN echo "source $FOREST_WS/setup.bash" >> ~/.bashrc

# Add recipes to forest workspace
RUN forest add-recipes https://github.com/ADVRHumanoids/multidof_recipes.git --tag $RECIPES_BRANCH

# Install dependencies using forest
RUN --mount=type=ssh,uid=${USER_ID} forest grow concert_description -m ${MODE} -j${JOBS} --pwd ${USER_PWD} --clone-depth 1 \
    && forest grow iit-dagana-ros-pkg -m ${MODE} -j${JOBS} --pwd ${USER_PWD} --clone-depth 1 \
    && rm -fr $FOREST_WS/build

# Build local version of modular
WORKDIR ${FOREST_WS}/src/modular
COPY --chown=user:user . ${FOREST_WS}/src/modular
RUN pip install -e .

# Set entrypoint
CMD ./scripts/entrypoint.sh
