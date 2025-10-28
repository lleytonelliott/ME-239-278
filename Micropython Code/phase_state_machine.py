# State Machine for Phases of Motion

class PhaseStateMachine:
    def __init__(self):
        self.current_phase = "null"

        self.phase_handlers = {
            "null": self.null_state,
            "sitting": self.sitting,
            "sit to stand": self.sit2stand,
            "standing": self.standing,
            "stand to sit": self.stand2sit,
            "shutdown": self.shutdown,
        }

        self.valid_events = {
            "startup",
            "stand_threshold_crossed",
            "finished_standing",
            "sit_threshold_crossed",
            "finished_sitting",
            "emergency",
            "startup_sitting",
            "startup_standing"
        }
        
    def transition(self, event):
        if event not in self.valid_events:
            print(f"Ignored Invalid Event '{event}'.")
            return
        
        handler = self.phase_handlers[self.current_phase]
        new_phase = handler(event)

        if self.current_phase != new_phase:
            print(f"Transition: '{self.current_phase}' -> '{new_phase}' on event '{event}'.")
            self.current_phase = new_phase
        
    def sitting(self, event, **kwargs):
        if event == "stand_threshold_crossed":
            return "sit to stand"
        elif event == "emergency":
            return "shutdown"
        return "sitting"
    
    def sit2stand(self, event, **kwargs):
        if event == "finished_standing":
            return "standing"
        elif event == "finished_sitting":
            return "sitting"
        elif event == "emergency":
            return "shutdown"
        return "sit to stand"
    
    def standing(self, event, **kwargs):
        if event == "sit_threshold_crossed":
            return "stand to sit"
        elif event == "emergency":
            return "shutdown"
        return "standing"

    def stand2sit(self, event, **kwargs):
        if event == "finished_sitting":
            return "sitting"
        elif event == "finished_standing":
            return "standing"
        elif event == "emergency":
            return "shutdown"
        return "stand to sit"
    
    def null_state(self, event, **kwargs):
        if event == "startup_sitting":
            return "sitting"
        elif event == "startup_standing":
            return "standing"
        elif event == "emergency":
            return "shutdown"
        return "null"
    
    def shutdown(self, event, **kwargs):
        if event == "startup":
            return "null"
        return "shutdown"