import * as React from "react";
import { Switch, Route } from "react-router-dom";
import HomeContainer from "./containers/home";

export default class Routes extends React.Component {
  public render() {
    return (
      <Switch>
        <Route exact path="*" component={HomeContainer} />
      </Switch>
    );
  }
}
