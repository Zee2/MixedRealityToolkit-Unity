// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using Microsoft.MixedReality.Toolkit.Utilities;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Microsoft.MixedReality.Toolkit.Physics
{
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
        [Serializable]
        public struct ElasticLinearExtentProperties {
            public float minStretch;       // Minimum stretch extent
            public float maxStretch;       // Maximum stretch extent
            public float[] demarcations;   // Points along the extent which the elastic will snap to
        }

        /// <summary>
        /// Properties of the damped harmonic oscillator differential system.
        /// </summary>
        [Serializable]
        public struct ElasticProperties
        {
            float mass;     // Mass of the simulated oscillator element
            float k;        // Spring constant
            float drag;     // Drag/damper factor, proportional to velocity.
        }

        ///// <summary>
        ///// Represents the mutable state of an elastic handle governed by a
        ///// damped harmonic oscillator differential system.
        ///// 
        ///// This is a struct instead of class, to prevent hammering the heap
        ///// whenever we call Setup() with new pointers.
        ///// 
        ///// </summary>
        //private struct ElasticHandle
        //{
        //    public Vector3 startPosition;
        //    public float x_initial;
        //    public float x_current;
        //    public float v_initial;
        //    public float v_current;
        //}

        private float currentLength = 0.1f;
        private float currentVelocity;

        private Vector3 leftInitialPosition;
        private Vector3 rightInitialPosition;

        private ElasticLinearExtentProperties extentInfo;
        private ElasticProperties elasticProperties;

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
                                    ElasticLinearExtentProperties extentInfo, ElasticProperties elasticProperties,
                                    float leftHandleVelocity = 0.0f, float rightHandleVelocity = 0.0f)
        {
            var center = (leftHandleStart + rightHandleStart) / 2.0f;
            var leftPos = Vector3.Magnitude(leftHandleStart - center);
            var rightPos = Vector3.Magnitude(rightHandleStart - center);
            isSetup = true;
            // Note; does not actually allocate on the heap.
            // ElasticHandle is a struct/value type, which is
            // mutated in place. Mutable structs are generally
            // considered poor design as it is easy to "lose changes"
            // when passing them around by value; take care to
            // avoid passing these to other functions, as they will
            // be copied.
            //leftHandle = new ElasticHandle
            //{
            //    startPosition = leftHandleStart,
            //    x_initial = leftPos,
            //    x_current = leftPos,
            //    v_initial = leftHandleVelocity,
            //    v_current = leftHandleVelocity

            //};
            //rightHandle = new ElasticHandle
            //{
            //    startPosition = rightHandleStart,
            //    x_initial = rightPos,
            //    x_current = rightPos,
            //    v_initial = rightHandleVelocity,
            //    v_current = rightHandleVelocity

            //};

            //currentPosition = Vector3.Magnitude(leftHandleStart - rightHandleStart);

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
            if (!isSetup) { return extentInfo.maxStretch; }

            var handDistance = currentLength;
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
            var force = (handDistance - currentLength) * 2.0f - 0.1f * currentVelocity;

            // Distance that the current stretch length is from the end cap.
            var distFromEnd = extentInfo.maxStretch - currentLength;

            // If we are extended beyond the end cap,
            // add one-sided force back to the center.
            if(currentLength > extentInfo.maxStretch)
            {
                force += distFromEnd * 5.0f;
            } else 
            {
                // Otherwise, add standard bidirectional magnetic/snapping force towards the end marker. (optional)
                //force += (distFromEnd) * 5.0f * (1.0f - Mathf.Clamp01(Mathf.Abs(15.0f * distFromEnd)));
            }

            // Iterate over each snapping point, and apply forces as necessary.
            foreach(float snappingPoint in extentInfo.demarcations)
            {
                // Calculate distance from snapping point.
                var distFromSnappingPoint = snappingPoint - currentLength;
                force += (distFromSnappingPoint) * 5.0f * (1.0f - Mathf.Clamp01(Mathf.Abs(15.0f * distFromSnappingPoint)));
            }

            // a = F/m
            var accel = force / 0.01f;

            // Integrate our acceleration over time.
            currentVelocity += accel * Time.deltaTime;
            // Integrate our velocity over time.
            currentLength += currentVelocity * Time.deltaTime;

            return currentLength;
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
