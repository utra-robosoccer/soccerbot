let allowPeriodicFetch = true;

export const getAllowPeriodicFetch = () => allowPeriodicFetch;

export const setAllowPeriodicFetch = (change: boolean) =>
  (allowPeriodicFetch = change);
