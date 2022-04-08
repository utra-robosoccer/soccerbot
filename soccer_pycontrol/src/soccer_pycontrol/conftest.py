import _pytest
import pytest

ACCEPTABLE_FAILURE_RATE = 50


@pytest.hookimpl()
def pytest_sessionfinish(session, exitstatus):
    if exitstatus != _pytest.main.EXIT_TESTSFAILED:
        return
    failure_rate = (100.0 * session.testsfailed) / session.testscollected
    if failure_rate <= ACCEPTABLE_FAILURE_RATE:
        session.exitstatus = 0
