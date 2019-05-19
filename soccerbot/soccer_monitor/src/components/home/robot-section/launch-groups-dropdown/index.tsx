import * as React from "react";
import "./index.scss";
import { map, isNil } from "lodash";

interface ILaunchConfigsDropdown {
  onClick: () => void;
  launchGroupOptions: Array<{ name: string; value: string }> | undefined;
  selectedLaunchGroupIndex: number;
  launchGroupDropdownOpen: boolean;
  iteratee: (v: { name: string }, index: number) => JSX.Element;
}

function LaunchGroupsDropdown(props: ILaunchConfigsDropdown) {
  return (
    <React.Fragment>
      <div
        data-value="launch-groups-dropdown"
        className="launch-groups-dropdown-selected"
        onClick={props.onClick}
      >
        <div data-value="launch-groups-dropdown" className="text">
          {!isNil(props.launchGroupOptions) &&
          props.launchGroupOptions!.length > 0
            ? props.launchGroupOptions![props.selectedLaunchGroupIndex].name
            : null}
        </div>
        <div data-value="launch-groups-dropdown" className="dropdown-icon">
          &#9660;
        </div>
      </div>
      {props.launchGroupDropdownOpen && (
        <div className="launch-groups-dropdown-list">
          {map(props.launchGroupOptions, props.iteratee)}
        </div>
      )}
    </React.Fragment>
  );
}

export default LaunchGroupsDropdown;
