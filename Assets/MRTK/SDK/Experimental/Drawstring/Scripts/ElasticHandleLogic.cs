// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using Microsoft.MixedReality.Toolkit.Utilities;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Microsoft.MixedReality.Toolkit.Physics
{

    /// <summary>
    /// Properties of the extent in which a damped
    /// harmonic oscillator is free to move.
    /// </summary>
    [Serializable]
    public struct ElasticExtentProperties
    {
        /// <value>
        /// Represents the lower bound of the extent,
        /// specified as the norm of the n-dimensional extent
        /// </value>
        [SerializeField]
        public float minStretch;

        /// <value>
        /// Represents the upper bound of the extent,
        /// specified as the norm of the n-dimensional extent
        /// </value>
        [SerializeField]
        public float maxStretch;

        /// <value>
        /// Whether the system, when approaching the upper bound,
        /// will treat the upper bound as a snap point.
        /// </value>
        [SerializeField]
        public bool snapToMax;

        /// <value>
        /// Points inside the extent to which the system will snap.
        /// </value>
        [SerializeField]
        public float[] snapPoints;
    }

    /// <summary>
    /// Properties of the damped harmonic oscillator differential system.
    /// </summary>
    [Serializable]
    public struct ElasticProperties
    {
        public float mass;          // Mass of the simulated oscillator element
        public float hand_k;        // Hand spring constant
        public float end_k;
        public float snap_k;        // Snap point spring constant
        public float snap_radius;   // Extent at which snap points begin forcing the spring.
        public float drag;          // Drag/damper factor, proportional to velocity.
    }

    /// <summary>
    /// Implements a two-handle elastic "stretch" logic, which allows for
    /// either one or two pointers to stretch along a particular axis.
    /// 
    /// Built around the differential equations for a damped harmonic oscillator.
    /// Intended for use with critically and over-damped oscillators.
    /// 
    /// Usage:
    /// When a manipulation starts, call Setup.
    /// Call Update any time to update the move logic and get a new rotation for the object.
    /// </summary>
    internal class ElasticHandleLogic
    {
        private Vector3 leftInitialPosition;
        private Vector3 rightInitialPosition;

        private ElasticExtentProperties extentInfo;
        private ElasticProperties elasticProperties;

        private float currentValue = 0.2f;
        private float currentVelocity;

        private bool isSetup;

        /// <summary>
        /// Initialize system with source info from controllers/hands.
        /// Left and right handles must both be specified, but this does not
        /// preclude single-hand interaction when Update() is called.
        /// </summary>
        /// <param name="leftHandleStart">World position of "left" handle point.</param>
        /// <param name="rightHandleStart">World position of "right" handle point.</param>
        /// <param name="extentInfo">Properties of the linear 1-D elastic extent to be manipulated</param>
        /// <param name="elasticProperties">Properties of the elastic material/spring.</param>
        /// <param name="leftHandleVelocity">Optional, initial velocity in 1-dimensional stretch space</param>
        /// <param name="rightHandleVelocity">Optional, initial velocity in 1-dimensional stretch space</param>
        public virtual void Setup(Vector3 leftHandleStart, Vector3 rightHandleStart,
                                    ElasticExtentProperties extentInfo, ElasticProperties elasticProperties,
                                    float leftHandleVelocity = 0.0f, float rightHandleVelocity = 0.0f)
        {
            isSetup = true;

            leftInitialPosition = leftHandleStart;
            rightInitialPosition = rightHandleStart;
            this.extentInfo = extentInfo;
            this.elasticProperties = elasticProperties;
        }

        /// <summary>
        /// Update the internal state of the damped harmonic oscillator, given the left and right pointer positions.
        /// Note, both left and right input positions are nullable; set the pointer position to null
        /// if only one of the handles is being interacted with. Returns the calculated distance between the
        /// handles.
        /// 
        /// </summary>
        /// <param name="leftPointer">World position of the pointer manipulating the left handle.</param>
        /// <param name="rightPointer">World position of the pointer manipulating the right handle.</param>
        public virtual float Update(Vector3? leftPointer, Vector3? rightPointer)
        {
            // If we have not been Setup() yet, we extend the handles
            // to the max stretch.
            if (!isSetup) { return currentValue; }

            var handDistance = currentValue;
            if (leftPointer.HasValue && rightPointer.HasValue)
            {
                // If we have both pointers, our hand distance is easy; just the distance
                // between the pointers.
                handDistance = Vector3.Magnitude(leftPointer.Value - rightPointer.Value);
            } else if (!leftPointer.HasValue && rightPointer.HasValue)
            {
                // If we only have a right pointer, calculate the hand distance
                // as twice the distance of the right pointer from the center.
                handDistance = 2.0f * Vector3.Magnitude(rightPointer.Value - (leftInitialPosition + rightInitialPosition) / 2.0f);
            } else if (leftPointer.HasValue && !rightPointer.HasValue)
            {
                // If we only have a left pointer, calculate the hand distance
                // as twice the distance of the left pointer from the center.
                handDistance = 2.0f * Vector3.Magnitude(leftPointer.Value - (leftInitialPosition + rightInitialPosition) / 2.0f);
            }

            // F = -kx - (drag * v)
            var force = (handDistance - currentValue) * elasticProperties.hand_k - elasticProperties.drag * currentVelocity;

            // Distance that the current stretch value is from the end limit.
            float distFromEnd = extentInfo.maxStretch - currentValue;

            // If we are extended beyond the end cap,
            // add one-sided force back to the center.
            if (currentValue > extentInfo.maxStretch)
            {
                force += distFromEnd * elasticProperties.end_k;
            }
            else
            {
                // Otherwise, add standard bidirectional magnetic/snapping force towards the end marker. (optional)
                if (extentInfo.snapToMax)
                {
                    force += (distFromEnd) * elasticProperties.end_k * (1.0f - Mathf.Clamp01(Mathf.Abs(distFromEnd / elasticProperties.snap_radius)));
                }
            }

            distFromEnd = extentInfo.minStretch - currentValue;
            if (currentValue < extentInfo.minStretch)
            {
                force += distFromEnd * elasticProperties.end_k;
            }
            else
            {
                // Otherwise, add standard bidirectional magnetic/snapping force towards the end marker. (optional)
                if (extentInfo.snapToMax)
                {
                    force += (distFromEnd) * elasticProperties.end_k * (1.0f - Mathf.Clamp01(Mathf.Abs(distFromEnd / elasticProperties.snap_radius)));
                }
            }

            // Iterate over each snapping point, and apply forces as necessary.
            foreach (float snappingPoint in extentInfo.snapPoints)
            {
                // Calculate distance from snapping point.
                var distFromSnappingPoint = snappingPoint - currentValue;

                // Snap force is calculated by multiplying the "-kx" factor by
                // a clamped distance factor. This results in an overall
                // hyperbolic profile to the force imparted by the snap point.
                force += (distFromSnappingPoint) * elasticProperties.snap_k
                          * (1.0f - Mathf.Clamp01(Mathf.Abs(distFromSnappingPoint / elasticProperties.snap_radius)));
            }

            // a = F/m
            var accel = force / elasticProperties.mass;

            // Integrate our acceleration over time.
            currentVelocity += accel * Time.deltaTime;
            // Integrate our velocity over time.
            currentValue += currentVelocity * Time.deltaTime;

            return currentValue;
        }

        public virtual Vector3 GetCenterPosition(Vector3? leftPointer, Vector3? rightPointer)
        {
            if (leftPointer.HasValue && rightPointer.HasValue)
            {
                return (leftPointer.Value + rightPointer.Value) / 2.0f;
            }
            else if (!leftPointer.HasValue && rightPointer.HasValue)
            {
                return (rightPointer.Value + leftInitialPosition) / 2.0f;
            }
            else if (leftPointer.HasValue && !rightPointer.HasValue)
            {
                return (leftPointer.Value + rightInitialPosition) / 2.0f;
            } else
            {
                return Vector3.zero; // Failure case.
            }
        }

            private float GetMinDistanceBetweenHands(Vector3[] handsPressedArray)
        {
            var result = float.MaxValue;
            for (int i = 0; i < handsPressedArray.Length; i++)
            {
                for (int j = i + 1; j < handsPressedArray.Length; j++)
                {
                    var distance = Vector3.Distance(handsPressedArray[i], handsPressedArray[j]);
                    if (distance < result)
                    {
                        result = distance;
                    }
                }
            }
            return result;
        }
    }
}
