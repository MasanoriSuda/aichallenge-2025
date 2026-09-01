# make file inspired by https://roborovsky-racers.github.io/RoborovskyNote/
SHELL := /bin/bash

.PHONY: autoware-build autoware-vehicle autoware-simulator autoware-request-initialpose autoware-request-control  awsim-request-start awsim-request-reset autoware-driver-zenoh autoware-driver-zenoh-rosbag \
	simulator dev dev2 dev3 dev4 e2e-single e2e-teacher e2e-npc-single e2e-npc-gap-teacher e2e-peer-audit-mpc e2e-peer-audit-student e2e e2e-final e2e-final-contact-teacher e2e-final-precontact-teacher e2e-final-precontact-teacher-all driver zenoh download rviz2 down down_all ps autoware-attach autoware-bash eval

# Used by docker-compose.yml for build/eval artifact ownership.
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
export HOST_UID HOST_GID
# Stop host shell's ROS_DOMAIN_ID from overriding .env via compose interpolation,
# but still honor an explicit `make foo ROS_DOMAIN_ID=N` command-line override.
unexport ROS_DOMAIN_ID
ifeq ($(origin ROS_DOMAIN_ID),command line)
export ROS_DOMAIN_ID
endif

TIMESTAMP := $(shell date +%Y%m%d-%H%M%S)
LOG_DIR := /output/$(TIMESTAMP)

# make simulator-<mode>: <mode> は simulator_scripts/*.sh のファイル名
SIM_MODES := $(notdir $(basename $(wildcard aichallenge/simulator_scripts/*.sh)))
# dev<N>（車両数）/ gate<N>（テスト番号）は run_simulator.bash が展開するエイリアス
SIM_MODES += dev2 dev3 dev4 gate1 gate2 gate3
.PHONY: $(addprefix simulator-,$(SIM_MODES))
$(addprefix simulator-,$(SIM_MODES)): simulator-%:
	@$(MAKE) simulator SIM_MODE=$*

# autowareのbuildのみ
autoware-build:
	docker compose run -T --rm --no-deps autoware-build

# run autoware for vehicle
autoware-vehicle:
	@echo "Start Autoware for Vehicle"
	LOG_DIR=$(LOG_DIR) RUN_MODE=vehicle docker compose up -d autoware

# run autoware for simulator
autoware-simulator:
	@echo "Start Autoware for AWSIM"
	LOG_DIR=$(LOG_DIR) RUN_MODE=awsim AIC_VEHICLE_COUNT=$(AIC_VEHICLE_COUNT) AIC_CONTROL_METHOD=$(AIC_CONTROL_METHOD) TINY_LIDAR_CKPT_PATH=$(TINY_LIDAR_CKPT_PATH) TINY_LIDAR_RESIDUAL_CKPT_PATH=$(TINY_LIDAR_RESIDUAL_CKPT_PATH) TINY_LIDAR_RESIDUAL_ARCHITECTURE=$(TINY_LIDAR_RESIDUAL_ARCHITECTURE) TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH=$(TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH) TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=$(TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED) TINY_LIDAR_CONTROL_MODE=$(TINY_LIDAR_CONTROL_MODE) docker compose up -d autoware

# autoware command service use ROS_DOMAIN_ID from .env
autoware-request-initialpose:
	CMD="ros2 service call /set_initial_pose std_srvs/srv/Trigger '{}'" docker compose run --rm --no-deps autoware-command

autoware-request-control:
	CMD="ros2 topic pub -1 /awsim/control_mode_request_topic std_msgs/msg/Bool '{data: true}'" docker compose run --rm --no-deps autoware-command

# awsim admin service use ROS_DOMAIN_ID 0
awsim-request-start:
	CMD="env ROS_DOMAIN_ID=0 ros2 topic pub -1 /admin/awsim/start std_msgs/msg/Bool '{data: true}'" docker compose run --rm --no-deps autoware-command

awsim-request-reset:
	CMD="env ROS_DOMAIN_ID=0 ros2 topic pub -1 /admin/awsim/reset std_msgs/msg/Empty '{}'" docker compose run --rm --no-deps autoware-command

# run simulator (docker compose up -d simulator)
simulator:
	@echo "Start AWSIM (SIM_MODE=$(SIM_MODE))"
	LOG_DIR=$(LOG_DIR) SIM_MODE="$(SIM_MODE)" E2E_START_RANDOM_SEED=$(E2E_START_RANDOM_SEED) ROS_DOMAIN_ID=0 docker compose up -d simulator

# racing kart (docker compose up -d driver)
driver:
	docker compose up -d driver

# zenoh (docker compose up -d zenoh)
zenoh:
	docker compose up -d zenoh

dev: SIM_MODE := dev
dev: AIC_VEHICLE_COUNT := 1
dev: simulator autoware-simulator
	@echo "Start dev simulation (AWSIM + Autoware)"
	@echo "To stop: make down  (docker compose down --remove-orphans)"

dev2: SIM_MODE := dev2
dev3: SIM_MODE := dev3
dev4: SIM_MODE := dev4
dev2 dev3 dev4: simulator
	@N=$(@:dev%=%); \
	echo "Start $$N-vehicle dev (autoware on ROS_DOMAIN_ID 1..$$N via docker compose -p)"; \
	for p in $$(seq 1 $$N); do LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p AIC_VEHICLE_COUNT=$$N docker compose -p $$p up -d autoware; done; \
	echo "To Stop: make down"

# End to End AI modes. e2e-single is the deterministic three-lap development gate;
# e2e/e2e-final mirror the upstream practice/final reference scenarios.
e2e-single: SIM_MODE := e2e-single
e2e-teacher: SIM_MODE := e2e-teacher
e2e-npc-single: SIM_MODE := e2e-npc-single
e2e-npc-gap-teacher: SIM_MODE := e2e-npc-single
e2e-peer-audit-mpc e2e-peer-audit-student: SIM_MODE := e2e-peer
e2e: SIM_MODE := e2e
e2e-final: SIM_MODE := e2e-final
e2e-final-contact-teacher: SIM_MODE := e2e-final
e2e-final-precontact-teacher: SIM_MODE := e2e-final
e2e-final-precontact-teacher-all: SIM_MODE := e2e-final
e2e-single e2e-npc-single e2e-npc-gap-teacher e2e e2e-final: AIC_CONTROL_METHOD := tiny_lidar_net
e2e-npc-gap-teacher: TINY_LIDAR_CONTROL_MODE := gap_teacher
e2e-teacher: AIC_CONTROL_METHOD := mpc
e2e-single e2e-teacher e2e: AIC_VEHICLE_COUNT := 1
# vehicle_count is the complete simulated world count for the launch contract,
# not the number of Autoware containers started by these one-ego targets. Counting
# both runtime NPCs prevents the single-vehicle empty-V2X producer from masking
# privileged teacher observations.
e2e-npc-single e2e-npc-gap-teacher: AIC_VEHICLE_COUNT := 3
e2e-single e2e-teacher e2e-npc-single e2e-npc-gap-teacher e2e: simulator autoware-simulator
	@echo "Start E2E simulation (SIM_MODE=$(SIM_MODE), controller=$(AIC_CONTROL_METHOD))"
	@echo "To stop: make down  (docker compose down --remove-orphans)"

e2e-peer-audit-mpc e2e-peer-audit-student: simulator
	@echo "Start deterministic 3-vehicle E2E peer gate (ego=domain 3, mode=$@)"
	@for p in 1 2; do \
		LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p AIC_VEHICLE_COUNT=3 AIC_CONTROL_METHOD=mpc \
			docker compose -p $$p up -d autoware; \
	done
	@LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=3 AIC_VEHICLE_COUNT=3 \
		AIC_CONTROL_METHOD=$(if $(filter e2e-peer-audit-student,$@),tiny_lidar_net,mpc) \
		TINY_LIDAR_CKPT_PATH=$(if $(filter e2e-peer-audit-student,$@),$(TINY_LIDAR_CKPT_PATH),) \
		docker compose -p 3 up -d autoware
	@echo "Audit only: current MPC peer runs are not admitted as E2E teacher data."
	@echo "Domain 2 is the configured low-speed peer; inspect the domain 3 bag."
	@echo "To stop: make down"

e2e-final: simulator
	@echo "Start 4-vehicle E2E final reference (Autoware on ROS_DOMAIN_ID 1..4)"
	@for p in $$(seq 1 4); do LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p AIC_VEHICLE_COUNT=4 AIC_CONTROL_METHOD=$(AIC_CONTROL_METHOD) docker compose -p $$p up -d autoware; done
	@echo "Start mode is sync; publish /admin/awsim/start after all vehicles are Ready."
	@echo "To stop: make down"

# Diagnostic-only final-world A/B.  Keep three production students unchanged and
# replace only domain 4 lateral authority with the existing admitted gap teacher.
# This target must never become the submission/default runtime path.
e2e-final-contact-teacher: simulator
	@echo "Start 4-vehicle E2E final contact-teacher audit (teacher domain=d4)"
	@for p in 1 2 3; do \
		LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p AIC_VEHICLE_COUNT=4 \
		AIC_CONTROL_METHOD=tiny_lidar_net TINY_LIDAR_CONTROL_MODE=fixed_lidar_brake \
		docker compose -p $$p up -d autoware; \
	done
	@LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=4 AIC_VEHICLE_COUNT=4 \
		AIC_CONTROL_METHOD=tiny_lidar_net TINY_LIDAR_CONTROL_MODE=gap_teacher \
		docker compose -p 4 up -d autoware
	@echo "Audit only: d4 commands are teacher labels only after run-level admission."
	@echo "Start mode is sync; publish /admin/awsim/start after all vehicles are Grounded."
	@echo "To stop: make down"

# Successor diagnostic for the rejected residual teacher.  Domain 4 uses a
# clustered side-return projection; production domains and defaults stay frozen.
e2e-final-precontact-teacher: simulator
	@echo "Start 4-vehicle E2E final pre-contact-teacher audit (teacher domain=d4)"
	@for p in 1 2 3; do \
		LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p AIC_VEHICLE_COUNT=4 \
		AIC_CONTROL_METHOD=tiny_lidar_net TINY_LIDAR_CONTROL_MODE=fixed_lidar_brake \
		docker compose -p $$p up -d autoware; \
	done
	@LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=4 AIC_VEHICLE_COUNT=4 \
		AIC_CONTROL_METHOD=tiny_lidar_net TINY_LIDAR_CONTROL_MODE=precontact_teacher \
		docker compose -p 4 up -d autoware
	@echo "Audit only: pre-contact commands are not production authority."
	@echo "Start mode is sync; publish /admin/awsim/start after all vehicles are Grounded."
	@echo "To stop: make down"

# Run-level teacher admission.  Unlike the isolated d4 audit, all peers use the
# same diagnostic policy so a known production contact trap cannot block Finish.
e2e-final-precontact-teacher-all: simulator
	@echo "Start 4-vehicle E2E final all-pre-contact-teacher admission"
	@for p in 1 2 3 4; do \
		LOG_DIR=$(LOG_DIR) ROS_DOMAIN_ID=$$p AIC_VEHICLE_COUNT=4 \
		AIC_CONTROL_METHOD=tiny_lidar_net TINY_LIDAR_CONTROL_MODE=precontact_teacher \
		docker compose -p $$p up -d autoware; \
	done
	@echo "Teacher labels remain inadmissible until every run-level gate passes."
	@echo "Start mode is sync; publish /admin/awsim/start after all vehicles are Grounded."
	@echo "To stop: make down"

gate1: SIM_MODE := gate1
gate2: SIM_MODE := gate2
gate3: SIM_MODE := gate3
gate1 gate2 gate3: AIC_VEHICLE_COUNT := 1
gate1 gate2 gate3: simulator autoware-simulator
	@echo "Start safety gate simulation (AWSIM + Autoware)"
	@echo "To stop: make down  (docker compose down --remove-orphans)"

eval:
	@echo "Start evaluation simulation (AWSIM + Autoware)"
	docker compose up -d autoware-simulator-evaluation
	$(MAKE) awsim-request-start
	@echo "To stop: make down  (docker compose down --remove-orphans)"

# remote operation (docker compose up -d rviz2)
rviz2:
	docker compose stop rviz2
	docker compose up -d rviz2

# driver + autoware + zenoh
autoware-driver-zenoh:
	LOG_DIR=$(LOG_DIR) RUN_MODE=vehicle docker compose up -d driver autoware
	sleep 15
	LOG_DIR=$(LOG_DIR) docker compose up -d zenoh

# driver + autoware + all-topic rosbag + zenoh
autoware-driver-zenoh-rosbag:
	LOG_DIR=$(LOG_DIR) RUN_MODE=vehicle docker compose up -d driver autoware rosbag
	sleep 15
	LOG_DIR=$(LOG_DIR) docker compose up -d zenoh

down:
	@for p in 1 2 3 4; do docker compose -p $$p down --remove-orphans; done
	@docker compose down --remove-orphans

down_all:
	sudo docker ps -aq | xargs -r sudo docker rm -f

ps:
	@docker compose ps
	@for p in 1 2 3 4; do \
		out=$$(docker compose -p $$p ps --format '{{.Name}}\t{{.Service}}\t{{.Status}}' 2>/dev/null); \
		if [ -n "$$out" ]; then \
			echo "--- project=$$p ---"; \
			echo "$$out"; \
		fi; \
	done

autoware-attach:
	@./docker_exec.sh

autoware-bash:
	CMD="bash --rcfile /etc/skel/.bashrc -i" docker compose run --rm --no-deps autoware-command

# Download submission data by asking for credentials interactively
# Usage:
#   make download [SUBMISSION_ID=<id>]
# Usage (Only Admins):
#   make download [USER_ID=<id>] [SUBMISSION_ID=<id>]
download:
	@if [ -n "$(USER_ID)" ]; then \
		if [ -n "$(SUBMISSION_ID)" ]; then \
			vehicle/download_submission.sh --output aichallenge/workspace/src/ --user-id $(USER_ID) --submission-id $(SUBMISSION_ID); \
		else \
			vehicle/download_submission.sh --output aichallenge/workspace/src/ --user-id $(USER_ID); \
		fi; \
	else \
		if [ -n "$(SUBMISSION_ID)" ]; then \
			vehicle/download_submission.sh --output aichallenge/workspace/src/ --submission-id $(SUBMISSION_ID); \
		else \
			vehicle/download_submission.sh --output aichallenge/workspace/src/; \
		fi; \
	fi
