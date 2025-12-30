"""
Graceful shutdown handling for long-running processes
"""
import signal
import sys
import time
import logging
import threading
from typing import Callable, Optional, Dict, Any
import atexit


logger = logging.getLogger(__name__)


class GracefulShutdown:
    """
    Class to handle graceful shutdown of long-running processes.
    """
    def __init__(self):
        self.shutdown_initiated = False
        self.original_sigint = None
        self.original_sigterm = None
        self.shutdown_callbacks = []
        self.lock = threading.Lock()

    def __enter__(self):
        self.initiate()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def initiate(self):
        """Initialize the graceful shutdown handler."""
        with self.lock:
            if not self.shutdown_initiated:
                # Register signal handlers
                self.original_sigint = signal.signal(signal.SIGINT, self._signal_handler)
                self.original_sigterm = signal.signal(signal.SIGTERM, self._signal_handler)

                # Register exit callback
                atexit.register(self._cleanup_on_exit)

                self.shutdown_initiated = True
                logger.info("Graceful shutdown handler initialized")

    def cleanup(self):
        """Cleanup the graceful shutdown handler."""
        with self.lock:
            if self.shutdown_initiated:
                # Restore original signal handlers
                if self.original_sigint:
                    signal.signal(signal.SIGINT, self.original_sigint)
                if self.original_sigterm:
                    signal.signal(signal.SIGTERM, self.original_sigterm)

                # Unregister exit callback
                atexit.unregister(self._cleanup_on_exit)

                self.shutdown_initiated = False
                logger.info("Graceful shutdown handler cleaned up")

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}, initiating graceful shutdown...")
        self._execute_shutdown_callbacks()
        sys.exit(0)

    def _cleanup_on_exit(self):
        """Cleanup function registered with atexit."""
        logger.info("Executing cleanup on exit...")
        self._execute_shutdown_callbacks()

    def _execute_shutdown_callbacks(self):
        """Execute all registered shutdown callbacks."""
        for callback in self.shutdown_callbacks:
            try:
                callback()
            except Exception as e:
                logger.error(f"Error in shutdown callback: {e}")

    def register_callback(self, callback: Callable[[], None]):
        """Register a callback function to be called during shutdown."""
        self.shutdown_callbacks.append(callback)
        logger.debug(f"Registered shutdown callback: {callback.__name__}")

    def is_shutting_down(self) -> bool:
        """Check if shutdown has been initiated."""
        return self.shutdown_initiated


# Global instance for convenience
graceful_shutdown = GracefulShutdown()


def wait_for_shutdown(shutdown_check_interval: float = 1.0):
    """
    Block execution and wait for shutdown signal.

    Args:
        shutdown_check_interval: How often to check for shutdown in seconds
    """
    try:
        while not graceful_shutdown.is_shutting_down():
            time.sleep(shutdown_check_interval)
    except KeyboardInterrupt:
        logger.info("Interrupted, initiating shutdown...")
        graceful_shutdown._execute_shutdown_callbacks()


def with_graceful_shutdown(func: Callable) -> Callable:
    """
    Decorator to add graceful shutdown handling to a function.

    Args:
        func: Function to wrap with graceful shutdown handling

    Returns:
        Wrapped function with graceful shutdown handling
    """
    def wrapper(*args, **kwargs):
        with GracefulShutdown():
            return func(*args, **kwargs)
    return wrapper


class LongRunningTaskManager:
    """
    Manager for long-running tasks with graceful shutdown capability.
    """
    def __init__(self):
        self.tasks = {}
        self.task_lock = threading.Lock()
        self.active_tasks = set()
        self.shutdown_handler = graceful_shutdown

    def register_task(self, task_id: str, task_func: Callable, *args, **kwargs):
        """
        Register a long-running task.

        Args:
            task_id: Unique identifier for the task
            task_func: Function to execute as the task
            *args: Arguments to pass to the task function
            **kwargs: Keyword arguments to pass to the task function
        """
        with self.task_lock:
            self.tasks[task_id] = {
                'function': task_func,
                'args': args,
                'kwargs': kwargs,
                'thread': None,
                'stop_event': threading.Event()
            }

            # Register a callback to stop this task on shutdown
            def stop_task():
                self._stop_task(task_id)

            self.shutdown_handler.register_callback(stop_task)
            logger.debug(f"Registered task: {task_id}")

    def _stop_task(self, task_id: str):
        """Stop a specific task."""
        with self.task_lock:
            if task_id in self.tasks:
                task_info = self.tasks[task_id]
                if task_info['stop_event']:
                    task_info['stop_event'].set()
                    logger.info(f"Stop signal sent to task: {task_id}")

    def start_task(self, task_id: str):
        """
        Start a registered task in a separate thread.

        Args:
            task_id: ID of the task to start
        """
        if task_id not in self.tasks:
            raise ValueError(f"Task {task_id} not registered")

        task_info = self.tasks[task_id]

        def task_wrapper():
            logger.info(f"Starting task: {task_id}")
            try:
                # Call the task function with stop_event as first parameter if it accepts it
                if task_info['function'].__code__.co_argcount > 0:
                    task_info['function'](task_info['stop_event'], *task_info['args'], **task_info['kwargs'])
                else:
                    task_info['function'](*task_info['args'], **task_info['kwargs'])
            except Exception as e:
                logger.error(f"Error in task {task_id}: {e}")
            finally:
                with self.task_lock:
                    if task_id in self.active_tasks:
                        self.active_tasks.remove(task_id)
                logger.info(f"Task completed: {task_id}")

        thread = threading.Thread(target=task_wrapper, daemon=True)
        task_info['thread'] = thread
        with self.task_lock:
            self.active_tasks.add(task_id)

        thread.start()
        logger.debug(f"Started task thread: {task_id}")

    def start_all_tasks(self):
        """Start all registered tasks."""
        for task_id in list(self.tasks.keys()):
            self.start_task(task_id)

    def wait_for_all_tasks(self, timeout: Optional[float] = None):
        """
        Wait for all tasks to complete.

        Args:
            timeout: Maximum time to wait in seconds, None for no timeout
        """
        start_time = time.time()

        while True:
            with self.task_lock:
                active_task_ids = list(self.active_tasks)

            if not active_task_ids:
                break

            if timeout is not None and (time.time() - start_time) >= timeout:
                logger.warning(f"Timeout waiting for tasks: {active_task_ids}")
                break

            time.sleep(0.1)  # Check every 100ms


def setup_pipeline_shutdown_handler():
    """
    Setup shutdown handling specifically for the pipeline application.
    """
    def cleanup_pipeline():
        """Cleanup function specific to the pipeline."""
        logger.info("Cleaning up pipeline resources...")
        # Add any pipeline-specific cleanup here
        # For example: close database connections, finalize file writes, etc.

    graceful_shutdown.register_callback(cleanup_pipeline)
    logger.info("Pipeline shutdown handler configured")


# Example usage functions
def example_long_running_task(stop_event: threading.Event, task_name: str = "example"):
    """
    Example of a long-running task that respects the stop event.

    Args:
        stop_event: Threading event to check for shutdown signals
        task_name: Name of the task for logging
    """
    logger.info(f"Starting long-running task: {task_name}")

    counter = 0
    while not stop_event.is_set():
        # Do some work
        logger.debug(f"{task_name} - Processing item {counter}")
        counter += 1

        # Check for stop event periodically
        if stop_event.wait(timeout=1.0):  # Wait 1 second or until stop event
            logger.info(f"Stop signal received for task: {task_name}")
            break

    logger.info(f"Long-running task completed: {task_name}")


def run_pipeline_with_shutdown_handling(urls: list, **kwargs):
    """
    Run the pipeline with graceful shutdown handling.

    Args:
        urls: List of URLs to process
        **kwargs: Additional arguments for the pipeline
    """
    logger.info("Starting pipeline with graceful shutdown handling")

    with GracefulShutdown():
        # Setup pipeline-specific shutdown handling
        setup_pipeline_shutdown_handler()

        try:
            # Import pipeline components
            from pipeline import PipelineOrchestrator

            # Initialize orchestrator
            orchestrator = PipelineOrchestrator(
                chunk_size=kwargs.get('chunk_size', 1000),
                chunk_overlap=kwargs.get('chunk_overlap', 200),
                cohere_model=kwargs.get('cohere_model'),
                collection_name=kwargs.get('collection_name')
            )

            # Run pipeline
            result = orchestrator.run_pipeline(urls)

            logger.info(f"Pipeline completed with result: {result['status']}")
            return result

        except KeyboardInterrupt:
            logger.info("Pipeline interrupted by user")
            return {'status': 'cancelled', 'error': 'Interrupted by user'}
        except Exception as e:
            logger.error(f"Pipeline failed: {e}")
            return {'status': 'failed', 'error': str(e)}


if __name__ == "__main__":
    # Example usage
    def test_shutdown():
        print("Testing graceful shutdown...")

        with GracefulShutdown():
            # Register a callback
            def my_cleanup():
                print("Performing cleanup...")

            graceful_shutdown.register_callback(my_cleanup)

            print("Press Ctrl+C to trigger shutdown...")
            wait_for_shutdown()

    # Uncomment the next line to test shutdown handling
    # test_shutdown()