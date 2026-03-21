import heapq
from enum import Enum, auto


class EventType(Enum):
    PACKET_TX_START = auto()
    PACKET_TX_END = auto()
    RX1_WINDOW_OPEN = auto()
    RX1_WINDOW_CLOSE = auto()
    RX2_WINDOW_OPEN = auto()
    RX2_WINDOW_CLOSE = auto()
    DUTY_CYCLE_RELEASE = auto()
    MOBILITY_UPDATE = auto()
    DEVICE_SEND = auto()
    JOIN_REQUEST = auto()
    SIMULATION_END = auto()
    BEACON = auto()
    ENERGY_HARVEST = auto()


class Event:
    """Representa um evento discreto na simulacao."""
    __slots__ = ('time', 'event_id', 'event_type', 'callback', 'args', 'cancelled')

    def __init__(self, time, event_id, event_type, callback, args):
        self.time = time
        self.event_id = event_id
        self.event_type = event_type
        self.callback = callback
        self.args = args
        self.cancelled = False

    def __lt__(self, other):
        return (self.time, self.event_id) < (other.time, other.event_id)


class EventScheduler:
    """Simulador de eventos discretos baseado em priority queue (heapq).
    Processa eventos em ordem cronologica estrita."""

    def __init__(self):
        self.event_queue = []
        self.current_time = 0.0
        self._next_id = 0
        self.events_processed = 0
        self.events_cancelled = 0
        self._compaction_threshold = 10000  # Compact after N cancelled events

    def schedule(self, delay, event_type, callback, *args):
        """Agenda um evento para current_time + delay segundos."""
        assert delay >= 0, f"Delay nao pode ser negativo: {delay}"
        event_time = self.current_time + delay
        event = Event(event_time, self._next_id, event_type, callback, args)
        self._next_id += 1
        heapq.heappush(self.event_queue, event)
        return event

    def schedule_absolute(self, time, event_type, callback, *args):
        """Agenda um evento em tempo absoluto."""
        assert time >= self.current_time, f"Nao pode agendar no passado: {time} < {self.current_time}"
        event = Event(time, self._next_id, event_type, callback, args)
        self._next_id += 1
        heapq.heappush(self.event_queue, event)
        return event

    def cancel(self, event):
        """Cancela um evento agendado (lazy cancellation)."""
        if event is not None:
            event.cancelled = True
            self.events_cancelled += 1

            # Compact periodicamente para evitar acumulo de eventos mortos
            if self.events_cancelled % self._compaction_threshold == 0:
                self.compact()

    def compact(self):
        """Remove eventos cancelados do heap."""
        self.event_queue = [e for e in self.event_queue if not e.cancelled]
        heapq.heapify(self.event_queue)

    def run(self, until):
        """Executa a simulacao processando eventos ate o tempo 'until'."""
        while self.event_queue:
            event = self.event_queue[0]
            if event.time > until:
                break
            heapq.heappop(self.event_queue)
            if event.cancelled:
                continue
            self.current_time = event.time
            event.callback(*event.args)
            self.events_processed += 1
        self.current_time = until

    @property
    def now(self):
        """Retorna o tempo atual da simulacao."""
        return self.current_time

    def pending_count(self):
        """Retorna o numero de eventos pendentes na fila."""
        return sum(1 for e in self.event_queue if not e.cancelled)

    def stats(self):
        """Retorna estatisticas do scheduler."""
        return {
            "events_processed": self.events_processed,
            "events_cancelled": self.events_cancelled,
            "events_pending": self.pending_count(),
            "current_time": self.current_time,
        }
