import * as React from "react";
import Home from "../../components/home";
import * as robotsInfoActionsUnbound from "../../actions/robots-info";
import * as robotsNotificationActionsUnbound from "../../actions/robots-notifications";
import { connect } from "react-redux";
import { AnyAction, bindActionCreators, Dispatch } from "redux";
import { IInitialState } from "../../reducers/initial-state";
import {
  RouteComponentProps as IRouteComponentProps,
  withRouter
} from "react-router";

interface IHomeContainerProps extends IRouteComponentProps {
  robots: IInitialState["robots"];
  robotsInfoActions: typeof robotsInfoActionsUnbound;
  robotsNotificationActions: typeof robotsNotificationActionsUnbound;
  robotsNotifications: IInitialState["robotsNotifications"];
  cmdGlobal: IInitialState["cmdGlobal"];
}

interface IHomeContainerState {}

class HomeContainer extends React.PureComponent<
  IHomeContainerProps,
  IHomeContainerState
> {
  public render() {
    const {
      robots,
      robotsInfoActions,
      robotsNotificationActions,
      robotsNotifications,
      cmdGlobal
    } = this.props;

    return (
      <Home
        robots={robots}
        robotsInfoActions={robotsInfoActions}
        robotsNotificationActions={robotsNotificationActions}
        robotsNotifications={robotsNotifications}
        cmdGlobal={cmdGlobal}
      />
    );
  }
}

function mapStateToProps(state: IInitialState) {
  return {
    robots: state.robots,
    robotsNotifications: state.robotsNotifications,
    cmdGlobal: state.cmdGlobal
  };
}

function mapDispatchToProps(dispatch: Dispatch<AnyAction>) {
  return {
    robotsInfoActions: bindActionCreators(robotsInfoActionsUnbound, dispatch),
    robotsNotificationActions: bindActionCreators(robotsNotificationActionsUnbound, dispatch)
  };
}

export default withRouter(
  connect(
    mapStateToProps,
    mapDispatchToProps
  )(HomeContainer)
);
