# Documentation

###

```mermaid
%%{init: {"classDiagram": {"useMaxWidth": false}} }%%
classDiagram
    direction LR
    namespace Detection {
        class ObjectDetectionNode
        class ObjectDetectionNodeRos

    }

    namespace Camera {
        class CameraBase
        class CameraCalculations
        class CameraCalculationsRos

    }

%%  Detector <|-- DetectorFieldline
    ObjectDetectionNode <|-- ObjectDetectionNodeRos
%%    DetectorFieldlineRos --|> DetectorFieldline

%%    CameraCalculations --|> CameraBase
    CameraBase <|-- CameraCalculations

    CameraCalculations <|-- CameraCalculationsRos

%%    CameraCalculations <|--* DetectorFieldline
    ObjectDetectionNode <|--* CameraCalculations

    ObjectDetectionNodeRos <|--* CameraCalculationsRos
%%    CameraCalculationsRos <|--* DetectorFieldlineRos




```
