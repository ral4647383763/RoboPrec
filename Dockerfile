FROM ubuntu:22.04

# Install sbt
RUN apt-get update
RUN apt-get install -y curl sudo
RUN echo "deb https://repo.scala-sbt.org/scalasbt/debian all main" | tee /etc/apt/sources.list.d/sbt.list
RUN echo "deb https://repo.scala-sbt.org/scalasbt/debian /" | tee /etc/apt/sources.list.d/sbt_old.list
RUN curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" | tee /etc/apt/trusted.gpg.d/sbt.asc
RUN apt-get update
RUN apt-get install -y sbt

# Install Java 8
RUN apt-get install -y openjdk-8-jdk

COPY daisy/ daisy

RUN cd daisy && \
    sbt update && \
    sbt compile && \
    sbt script

COPY quanta/ quanta

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable

RUN apt-get install -y g++

RUN apt-get install -y python3-pip
RUN pip3 install numpy pandas matplotlib




