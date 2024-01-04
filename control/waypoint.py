
class WaypointsLoader(object):
    def __init__(self, waypoints: list, threshold: float = 0.5) -> None:
        self.waypoints: list = waypoints if waypoints else []
        self.threshold: float = threshold

    def __iter__(self) -> "WaypointsLoader":
        return self
    
    def __next__(self) -> None:
        pass

    def update(self) -> bool:
        pass

if __name__ == "__main__":
    wps: list[tuple] = [
        (0, 0, 0, 0), 
        (0, 0, 1, 0),
        (1, 1, 1, 0),
        (1, 1, 1, 1),
    ]

    loader = WaypointsLoader(wps)
