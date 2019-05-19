import * as React from "react";
import Routes from "./Routes";
import "./global-styles/index.scss";

export default class App extends React.Component<{}, {}> {
  public render() {
    return (
      <React.Fragment>
        <Routes />
      </React.Fragment>
    );
  }
}
