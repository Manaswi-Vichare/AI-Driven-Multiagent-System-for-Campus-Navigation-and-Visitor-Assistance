from pydantic import BaseModel, Field
from typing import Optional

class Task:
    def __init__(self, description, agent, type, expected_output):
        self.description = description
        self.agent = agent
        self.type = type
        self.expected_output = expected_output

    def execute(self):
        # Logic for executing the task
        if hasattr(self.agent, 'perform_task'):
            self.agent.perform_task(self)
        else:
            raise AttributeError(f"{type(self.agent).__name__} object has no method 'perform_task'")
