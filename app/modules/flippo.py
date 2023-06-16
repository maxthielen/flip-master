import asyncio
from functools import partial
import numpy as np

from ..interfaces.flipper import flipper


class Flippo(flipper):
    def __init__(self) -> None:
        super().__init__()

    def raise_pistons():
        pass

    def lower_pistons():
        pass

    async def trigger_flip(self):
        loop = asyncio.get_event_loop()
        action_done = loop.create_future()

        def done_raise_pistons(result, future, loop):
            loop.call_soon_threadsafe(future.set_result(result))

        def done_lower_pistons(result, future, loop):
            loop.call_soon_threadsafe(future.set_result(result))

        action.send_goal(goal, partial(done_raise_pistons, future=action_done, loop=loop))

        try:
            await action_done()
        except asyncio.CancelledError as exc:
            action.cancel()
            raise exc

        return action_done.result()
        

        if np.array_equal(piston_sensors, np.array([1,0,1,0])) and self.robot.lowering_piston:
            self.lowering_piston = False
            return
        elif np.array_equal(piston_sensors, np.array([1,0,1,0])):
            self.raise_pistons()
        elif np.array_equal(piston_sensors, np.array([0,1,0,1])):
            self.flipper.lowering_piston = True
            self.lower_pistons()
        else:
            raise Exception("Unexpected piston sensor state")

    def _generate_action_executor(action):
        async def run_action(goal):
            loop = asyncio.get_event_loop()
            action_done = loop.create_future()

            def done_callback(goal_status, result, future, loop):
                status = ActionLibGoalStatus(goal_status)
                print('Action Done: {}'.format(status))
                loop.call_soon_threadsafe(future.set_result(result))

            action.send_goal(goal,partial(done_callback, future=action_done, loop=loop))
            try:
                await action_done
            except asyncio.CancelledError as exc:
                action.cancel()
                raise exc

            return action_done.result()

        return run_action

