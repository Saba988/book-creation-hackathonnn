"""
Pipeline progress reporting and metrics
"""
import time
from typing import Dict, Any, List
from datetime import datetime
import logging

from entities import PipelineState


logger = logging.getLogger(__name__)


class PipelineMetrics:
    """
    Class to track and report pipeline metrics and progress.
    """
    def __init__(self):
        self.start_time = None
        self.end_time = None
        self.steps = {}
        self.errors = []
        self.warnings = []

    def start_pipeline(self):
        """Record the start time of the pipeline."""
        self.start_time = time.time()
        logger.info("Pipeline metrics tracking started")

    def end_pipeline(self):
        """Record the end time of the pipeline."""
        self.end_time = time.time()
        logger.info("Pipeline metrics tracking ended")

    def start_step(self, step_name: str):
        """Record the start time of a pipeline step."""
        self.steps[step_name] = {
            'start_time': time.time(),
            'end_time': None,
            'duration': None
        }
        logger.debug(f"Started pipeline step: {step_name}")

    def end_step(self, step_name: str):
        """Record the end time of a pipeline step."""
        if step_name in self.steps:
            self.steps[step_name]['end_time'] = time.time()
            self.steps[step_name]['duration'] = (
                self.steps[step_name]['end_time'] - self.steps[step_name]['start_time']
            )
            logger.debug(f"Completed pipeline step: {step_name} in {self.steps[step_name]['duration']:.2f}s")

    def add_error(self, error_msg: str, step: str = None, url: str = None):
        """Add an error to the metrics tracker."""
        error = {
            'timestamp': datetime.now().isoformat(),
            'error': error_msg,
            'step': step,
            'url': url
        }
        self.errors.append(error)
        logger.error(f"Error in step '{step}' for URL '{url}': {error_msg}")

    def add_warning(self, warning_msg: str, step: str = None, url: str = None):
        """Add a warning to the metrics tracker."""
        warning = {
            'timestamp': datetime.now().isoformat(),
            'warning': warning_msg,
            'step': step,
            'url': url
        }
        self.warnings.append(warning)
        logger.warning(f"Warning in step '{step}' for URL '{url}': {warning_msg}")

    def get_pipeline_duration(self) -> float:
        """Get the total duration of the pipeline."""
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        return 0

    def get_step_duration(self, step_name: str) -> float:
        """Get the duration of a specific step."""
        if step_name in self.steps and self.steps[step_name]['duration']:
            return self.steps[step_name]['duration']
        return 0

    def get_total_steps_duration(self) -> float:
        """Get the sum of all step durations."""
        return sum(step.get('duration', 0) for step in self.steps.values() if step.get('duration'))

    def get_error_count(self) -> int:
        """Get the total number of errors."""
        return len(self.errors)

    def get_warning_count(self) -> int:
        """Get the total number of warnings."""
        return len(self.warnings)

    def get_metrics_summary(self) -> Dict[str, Any]:
        """Get a summary of all metrics."""
        total_pipeline_time = self.get_pipeline_duration()
        total_steps_time = self.get_total_steps_duration()
        unused_time = total_pipeline_time - total_steps_time

        return {
            'pipeline_duration': total_pipeline_time,
            'steps_duration': total_steps_time,
            'overhead_duration': unused_time,
            'total_errors': self.get_error_count(),
            'total_warnings': self.get_warning_count(),
            'steps': {
                name: {
                    'duration': step['duration'],
                    'start_time': step['start_time'],
                    'end_time': step['end_time']
                }
                for name, step in self.steps.items()
                if step.get('duration')
            },
            'errors': self.errors,
            'warnings': self.warnings
        }

    def log_summary(self):
        """Log a summary of the metrics."""
        summary = self.get_metrics_summary()

        logger.info("=" * 50)
        logger.info("PIPELINE METRICS SUMMARY")
        logger.info("=" * 50)
        logger.info(f"Total Pipeline Duration: {summary['pipeline_duration']:.2f}s")
        logger.info(f"Steps Duration: {summary['steps_duration']:.2f}s")
        logger.info(f"Overhead Duration: {summary['overhead_duration']:.2f}s")
        logger.info(f"Total Errors: {summary['total_errors']}")
        logger.info(f"Total Warnings: {summary['total_warnings']}")

        for step_name, step_data in summary['steps'].items():
            logger.info(f"  {step_name}: {step_data['duration']:.2f}s")

        if summary['errors']:
            logger.info(f"  Sample Error: {summary['errors'][0]['error'] if summary['errors'] else 'None'}")

        if summary['warnings']:
            logger.info(f"  Sample Warning: {summary['warnings'][0]['warning'] if summary['warnings'] else 'None'}")

        logger.info("=" * 50)


def track_pipeline_progress(pipeline_func):
    """
    Decorator to automatically track metrics for pipeline functions.
    """
    def wrapper(*args, **kwargs):
        metrics = PipelineMetrics()
        metrics.start_pipeline()

        try:
            result = pipeline_func(*args, **kwargs)
            return result
        finally:
            metrics.end_pipeline()
            metrics.log_summary()

    return wrapper


def report_pipeline_state(state: PipelineState) -> str:
    """
    Generate a human-readable report of the pipeline state.

    Args:
        state: The pipeline state to report

    Returns:
        Formatted string report of the pipeline state
    """
    report = f"""
PIPELINE STATE REPORT
====================
Status: {state.status}
Progress: {state.progress or 0:.1f}%
Processed URLs: {len(state.processed_urls)}
Failed URLs: {len(state.failed_urls)}
Total Chunks: {state.total_chunks or 0}
Timestamp: {state.timestamp.isoformat() if state.timestamp else 'N/A'}

Processed URLs:
"""
    for url in state.processed_urls[:5]:  # Show first 5
        report += f"  - {url}\n"

    if len(state.processed_urls) > 5:
        report += f"  ... and {len(state.processed_urls) - 5} more\n"

    if state.failed_urls:
        report += "\nFailed URLs:\n"
        for failure in state.failed_urls[:5]:  # Show first 5
            report += f"  - {failure['url']}: {failure['error']}\n"

        if len(state.failed_urls) > 5:
            report += f"  ... and {len(state.failed_urls) - 5} more\n"

    return report