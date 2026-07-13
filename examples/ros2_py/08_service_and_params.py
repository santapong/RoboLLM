#!/usr/bin/env python3
"""08 · Services + Parameters — the two ROS 2 primitives you haven't used yet.

Topics stream (01–05), actions run long jobs (06–07). SERVICES are one-shot
request→reply ("add these numbers", "reset the map", "switch the controller"),
and PARAMETERS are a node's runtime-tunable config.

This file is self-contained: it starts a server node AND a client node, calls
the service, changes a parameter, and shows the parameter change the behavior.

    .venv/bin/python examples/ros2_py/08_service_and_params.py

While it runs (or in your own nodes) you can also poke it from the CLI:
    ros2 service list
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
    ros2 param get /adder_server offset
    ros2 param set /adder_server offset 100
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from example_interfaces.srv import AddTwoInts


class AdderServer(Node):
    """Serves /add_two_ints. Result = a + b + <offset parameter>."""

    def __init__(self):
        super().__init__("adder_server")
        # declare_parameter gives it a default AND makes it visible/settable
        self.declare_parameter("offset", 0)
        # optional: get told when someone changes a parameter
        self.add_on_set_parameters_callback(self.on_param_change)
        # NOTE: don't name the callback `handle` — that would shadow Node.handle!
        self.create_service(AddTwoInts, "add_two_ints", self.on_request)
        self.get_logger().info("service /add_two_ints ready (param: offset)")

    def on_param_change(self, params):
        for p in params:
            self.get_logger().info(f"parameter changed: {p.name} = {p.value}")
        return SetParametersResult(successful=True)

    def on_request(self, request, response):
        offset = self.get_parameter("offset").value
        response.sum = request.a + request.b + offset
        self.get_logger().info(f"{request.a} + {request.b} (+offset {offset}) = {response.sum}")
        return response


def call(client, a, b):
    req = AddTwoInts.Request()
    req.a, req.b = a, b
    future = client.call_async(req)          # non-blocking; executor completes it
    while not future.done():
        time.sleep(0.02)
    return future.result().sum


def main():
    rclpy.init()
    server = AdderServer()
    caller = rclpy.create_node("adder_client")

    # one executor spins BOTH nodes in the background
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(caller)
    threading.Thread(target=executor.spin, daemon=True).start()

    client = caller.create_client(AddTwoInts, "add_two_ints")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("service never appeared")

    print("\ncall 1:  2 + 3        ->", call(client, 2, 3))          # 5

    # change the server's parameter, same service now behaves differently
    server.set_parameters([Parameter("offset", value=100)])
    print("call 2:  2 + 3 (+100) ->", call(client, 2, 3))            # 105

    ok = call(client, 2, 3) == 105
    print("\nRESULT:", "PASS — service replied and the parameter changed behavior" if ok else "FAIL")
    executor.shutdown()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
