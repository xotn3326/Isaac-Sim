# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import asyncio
import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import GripperUR
import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder, str_builder

class GripperURExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="UR10",
            submenu_name="",
            name="Gripper UR10",
            title="Gripper UR10 Control",
            doc_link="",
            overview="This Task shows how to follow a target using UR10 with Robotiq Gripper in Isaac Sim.",
            file_path=os.path.abspath(__file__),
            sample=GripperUR(),
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        return

    def _on_follow_target_button_event(self):
        asyncio.ensure_future(self.sample._on_follow_target_event_async())
        self.task_ui_elements["Follow Target"].enabled = True
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Follow Target"].enabled = True
        return

    def post_load_button_event(self):
        self.task_ui_elements["Follow Target"].enabled = True
        return

    def shutdown_cleanup(self):
        return

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                dict = {
                    "label" : "Follow Target",
                    "type" : "button",
                    "text" : "Follow Target",
                    "tooltip" : "Follow Target",
                    "on_clicked_fn": self._on_follow_target_button_event,
                }

                self.task_ui_elements["Follow Target"] = btn_builder(**dict)
                self.task_ui_elements["Follow Target"].enabled = False
