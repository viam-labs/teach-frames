// Package worldstatestore implements the teach-mode world_state_store mirror.
package worldstatestore

import (
	"github.com/google/uuid"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"google.golang.org/protobuf/types/known/structpb"
)

// namespaceUUID is a fixed namespace so name->UUID is stable across runs/restarts.
var namespaceUUID = uuid.MustParse("6f0e1a3c-0000-4000-8000-000000000000")

// uuidForName derives a stable UUIDv5 (binary form) from a frame name.
func uuidForName(name string) []byte {
	u := uuid.NewSHA1(namespaceUUID, []byte(name))
	b, _ := u.MarshalBinary()
	return b
}

// frameToTransform maps a taught frame to a world_state_store Transform with an
// axes-helper hint so the visualization renders it as a coordinate triad.
func frameToTransform(name string, pif *referenceframe.PoseInFrame) *commonpb.Transform {
	meta, _ := structpb.NewStruct(map[string]interface{}{"showAxesHelper": true})
	return &commonpb.Transform{
		ReferenceFrame:      name,
		PoseInObserverFrame: referenceframe.PoseInFrameToProtobuf(pif),
		Metadata:            meta,
		Uuid:                uuidForName(name),
	}
}
