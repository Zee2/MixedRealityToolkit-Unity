﻿// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using Microsoft.MixedReality.Toolkit.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace Microsoft.MixedReality.Toolkit.SceneSystem
{
    /// <summary>
    /// The default implementation of the <see cref="Microsoft.MixedReality.Toolkit.SceneSystem.IMixedRealitySceneSystem"/>
    /// Because so much of this service's functionality is editor-only, it has been split into a partial class.
    /// This part handles the runtime parts of the service.
    /// </summary>
    [DocLink("https://microsoft.github.io/MixedRealityToolkit-Unity/Documentation/SceneSystem/SceneSystemGettingStarted.html")]
    public partial class MixedRealitySceneSystem : BaseCoreSystem, IMixedRealitySceneSystem
    {
        /// <summary>
        /// Async load operation progress amount indicating that we're ready to activate a scene.
        /// https://docs.unity3d.com/ScriptReference/AsyncOperation-progress.html
        /// </summary>
        const float SceneActivationLoadProgress = 0.9f;

        /// <summary>
        /// Used by internal load methods to decide which actions to invoke.
        /// </summary>
        private enum SceneType
        {
            Manager = 0,
            Content = 1,
            Lighting = 2,
        }

        public MixedRealitySceneSystem(
            IMixedRealityServiceRegistrar registrar,
            MixedRealitySceneSystemProfile profile) : base(registrar, profile)
        {
            this.profile = profile;
        }

        private MixedRealitySceneSystemProfile profile;

        // Internal scene operation info
        private bool managerSceneOpInProgress;
        private float managerSceneOpProgress;

        // Content tracker instance
        private SceneContentTracker contentTracker;
        // Lighting executor instance
        private SceneLightingExecutor lightingExecutor;

        #region Actions

        /// <inheritdoc />
        public Action<IEnumerable<string>> OnWillLoadContent { get; set; }

        /// <inheritdoc />
        public Action<IEnumerable<string>> OnContentLoaded { get; set; }

        /// <inheritdoc />
        public Action<IEnumerable<string>> OnWillUnloadContent { get; set; }

        /// <inheritdoc />
        public Action<IEnumerable<string>> OnContentUnloaded { get; set; }

        /// <inheritdoc />
        public Action<string> OnWillLoadLighting { get; set; }

        /// <inheritdoc />
        public Action<string> OnLightingLoaded { get; set; }

        /// <inheritdoc />
        public Action<string> OnWillUnloadLighting { get; set; }

        /// <inheritdoc />
        public Action<string> OnLightingUnloaded { get; set; }

        /// <inheritdoc />
        public Action<string> OnWillLoadScene { get; set; }

        /// <inheritdoc />
        public Action<string> OnSceneLoaded { get; set; }

        /// <inheritdoc />
        public Action<string> OnWillUnloadScene { get; set; }

        /// <inheritdoc />
        public Action<string> OnSceneUnloaded { get; set; }

        #endregion

        #region Properties

        /// <inheritdoc />
        public bool SceneOperationInProgress { get; private set; } = false;

        /// <inheritdoc />
        public float SceneOperationProgress { get; private set; } = 0;

        /// <inheritdoc />
        public bool LightingOperationInProgress { get; private set; } = false;

        /// <inheritdoc />
        public float LightingOperationProgress { get; private set; } = 0;

        /// <inheritdoc />
        public string ActiveLightingScene { get; private set; } = string.Empty;

        /// <inheritdoc />
        public bool WaitingToProceed { get; private set; } = false;

        /// <inheritdoc />
        public bool PrevContentExists => contentTracker.PrevContentExists;

        /// <inheritdoc />
        public bool NextContentExists => contentTracker.NextContentExists;

        /// <inheritdoc />
        public string[] ContentSceneNames => contentTracker.ContentSceneNames;

        /// <inheritdoc />
        public uint SourceId { get; } = 0;

        /// <inheritdoc />
        public string SourceName { get; } = "Mixed Reality Scene System";

        #endregion

        #region Service Methods

        /// <inheritdoc />
        public override void Initialize()
        {
            // Create a new instance of our content tracker
            contentTracker = new SceneContentTracker(profile);
            lightingExecutor = new SceneLightingExecutor();

#if UNITY_EDITOR
            EditorOnInitialize();
#endif

            if (!Application.isPlaying)
            {
                return;
            }

            if (profile.UseManagerScene)
            {
                SetManagerScene(profile.ManagerScene.Name);
            }

            if (profile.UseLightingScene)
            {   // Set our lighting scene immediately, with no transition
                SetLightingScene(profile.DefaultLightingScene.Name, LightingSceneTransitionType.None);
            }
        }

        /// <inheritdoc />
        public override void Enable()
        {
#if UNITY_EDITOR
            EditorOnDisable();
#endif
        }

        /// <inheritdoc />
        public override void Disable()
        {
#if UNITY_EDITOR
            EditorOnDisable();
#endif
        }

        /// <inheritdoc />
        public override void Destroy()
        {
#if UNITY_EDITOR
            EditorOnDestroy();
#endif
        }

        /// <inheritdoc />
        public override void Update()
        {
            // Ensure the lighting scene is active, if we're using one.
            if (profile.UseLightingScene)
            {
                lightingExecutor.UpdateTransition(Time.unscaledDeltaTime);
            }
        }

        #endregion

        #region Scene Operations

        /// <inheritdoc />
        public async Task LoadNextContent(bool wrap = false, LoadSceneMode mode = LoadSceneMode.Single, SceneActivationToken activationToken = null)
        {
            string nextContent;
            if (contentTracker.GetNextContent(wrap, out nextContent))
            {
                await LoadContent(new string[] { nextContent }, mode, activationToken);
            }
            else
            {
                Debug.LogWarning("Attempted to load next content when no next content exists. Taking no action.");
            }
        }

        /// <inheritdoc />
        public async Task LoadPrevContent(bool wrap = false, LoadSceneMode mode = LoadSceneMode.Single, SceneActivationToken activationToken = null)
        {
            string prevContent;
            if (contentTracker.GetPrevContent(wrap, out prevContent))
            {
                await LoadContent(new string[] { prevContent }, mode, activationToken);
            }
            else
            {
                Debug.LogWarning("Attempted to load prev content when no next content exists. Taking no action.");
            }
        }

        /// <inheritdoc />
        public async Task LoadContent(string sceneToLoad, LoadSceneMode mode = LoadSceneMode.Additive, SceneActivationToken activationToken = null)
        {
            await LoadContent(new string[] { sceneToLoad }, mode, activationToken);
        }

        /// <inheritdoc />
        public async Task UnloadContent(string sceneToUnload)
        {
            await UnloadContent(new string[] { sceneToUnload });
        }

        /// <inheritdoc />
        public async Task LoadContentByTag(string tag, LoadSceneMode mode = LoadSceneMode.Additive, SceneActivationToken activationToken = null)
        {
            await LoadContent(profile.GetContentSceneNamesByTag(tag), mode, activationToken);
        }

        /// <inheritdoc />
        public async Task UnloadContentByTag(string tag)
        {
            await UnloadScenesInternal(profile.GetContentSceneNamesByTag(tag), SceneType.Content);
        }

        /// <inheritdoc />
        public async Task LoadContent(IEnumerable<string> scenesToLoad, LoadSceneMode mode = LoadSceneMode.Additive, SceneActivationToken activationToken = null)
        {
            if (!CanSceneOpProceed(SceneType.Content))
            {
                Debug.LogError("Attempting to perform a scene op when a scene op is already in progress.");
                return;
            }

            IEnumerable<string> loadedContentScenes;
            if (mode == LoadSceneMode.Single && GetLoadedContentScenes(out loadedContentScenes))
            {
                await UnloadScenesInternal(loadedContentScenes, SceneType.Content, 0, 0.5f, true);
                await LoadScenesInternal(scenesToLoad, SceneType.Content, activationToken, 0.5f, 1f, false);
            }
            else
            {
                await LoadScenesInternal(scenesToLoad, SceneType.Content, activationToken);
            }

            await LoadScenesInternal(scenesToLoad, SceneType.Content, activationToken);
        }

        /// <inheritdoc />
        public async Task UnloadContent(IEnumerable<string> scenesToUnload)
        {
            if (!CanSceneOpProceed(SceneType.Content))
            {
                Debug.LogError("Attempting to perform a scene op when a scene op is already in progress.");
                return;
            }

            await UnloadScenesInternal(scenesToUnload, SceneType.Content);
        }

        /// <inheritdoc />
        public bool IsContentLoaded(string sceneName)
        {
            Scene scene = SceneManager.GetSceneByName(sceneName);
            return scene.IsValid() && scene.isLoaded;
        }

        /// <inheritdoc />
        public async void SetLightingScene(string newLightingSceneName, LightingSceneTransitionType transitionType = LightingSceneTransitionType.None, float transitionDuration = 1f)
        {
            Debug.Log("Set lighting scene: " + newLightingSceneName);

            if (ActiveLightingScene == newLightingSceneName)
            {   // Nothing to do here
                return;
            }

            if (!CanSceneOpProceed(SceneType.Lighting))
            {
                Debug.LogError("Attempting to perform a scene op when a scene op is already in progress.");
                return;
            }

            SceneInfo lightingScene;
            RuntimeLightingSettings lightingSettings = default(RuntimeLightingSettings);
            RuntimeRenderSettings renderSettings = default(RuntimeRenderSettings);
            RuntimeSunlightSettings sunSettings = default(RuntimeSunlightSettings);
            if (!string.IsNullOrEmpty(newLightingSceneName) && !profile.GetLightingSceneSettings(
                newLightingSceneName,
                out lightingScene, 
                out lightingSettings, 
                out renderSettings,
                out sunSettings))
            {   // Make sure we don't try to load a non-existent scene
                Debug.LogWarning("Couldn't find lighting scene " + newLightingSceneName + " in profile - taking no action.");
                return;
            }

            ActiveLightingScene = newLightingSceneName;

            if (!Application.isPlaying)
            {   // Everything else is runtime-only
                return;
            }

            // Start the lighting executor transition - don't bother waiting for load / unload, it can start right away
            lightingExecutor.StartTransition(lightingSettings, renderSettings, sunSettings, transitionType, transitionDuration);

            List<string> lightingSceneNames = new List<string>();
            // Create a list of lighting scenes to unload
            foreach (SceneInfo lso in LightingScenes)
            {
                if (lso.Name != newLightingSceneName)
                {
                    lightingSceneNames.Add(lso.Name);
                }
            }

            // Load the new lighting scene immediately
            await LoadScenesInternal(new string[] { newLightingSceneName }, SceneType.Lighting, null, 0f, 0.5f, true);

            // Unload the other lighting scenes
            await UnloadScenesInternal(lightingSceneNames, SceneType.Lighting, 0.5f, 1f, false);
        }

        /// <summary>
        /// Loads the manager scene.
        /// </summary>
        /// <param name="managerSceneName"></param>
        private async void SetManagerScene(string managerSceneName)
        {
            Scene scene = SceneManager.GetSceneByName(managerSceneName);
            if (scene.IsValid() && !scene.isLoaded)
            {   // If the manager scene is already loaded, don't proceed.
                return;
            }

            await LoadScenesInternal(new string[] { managerSceneName }, SceneType.Manager);
        }

        /// <summary>
        /// Internal method to handle scene loads
        /// </summary>
        /// <param name="scenesToLoad"></param>
        /// <param name="sceneType"></param>
        /// <param name="activationToken"></param>
        /// <param name="progressOffset"></param>
        /// <param name="progressTarget"></param>
        /// <param name="sceneOpInProgressWhenFinished"></param>
        /// <returns></returns>
        private async Task LoadScenesInternal(
            IEnumerable<string> scenesToLoad,
            SceneType sceneType,
            SceneActivationToken activationToken = null,
            float progressOffset = 0,
            float progressTarget = 1,
            bool sceneOpInProgressWhenFinished = false)
        {
            // If we're using an activation token let it know that we're NOT ready to proceed
            activationToken?.SetReadyToProceed(false);

            SetSceneOpProgress(true, progressOffset, sceneType);

            // Validate our scenes
            List<string> validNames = new List<string>();
            List<int> validIndexes = new List<int>();

            foreach (string sceneName in scenesToLoad)
            {
                // See if scene exists
                Scene scene;
                int sceneIndex;
                if (!RuntimeSceneUtils.FindScene(sceneName, out scene, out sceneIndex))
                {
                    Debug.LogError("Can't load invalid scene " + sceneName);
                }
                else
                {
                    validIndexes.Add(sceneIndex);
                    validNames.Add(sceneName);
                }
            }

            int totalSceneOps = validIndexes.Count;
            if (totalSceneOps < 1)
            {
                Debug.LogWarning("No valid scenes found to load.");
                SetSceneOpProgress(sceneOpInProgressWhenFinished, progressTarget, sceneType);
                return;
            }

            // We're about to load scenes - let everyone know
            InvokeWillLoadActions(validNames, sceneType);

            // Load our scenes
            if (validIndexes.Count > 0)
            {
                List<AsyncOperation> loadSceneOps = new List<AsyncOperation>();
                foreach (int sceneIndex in validIndexes)
                {
                    Scene scene = SceneManager.GetSceneByBuildIndex(sceneIndex);
                    if (scene.isLoaded)
                        continue;

                    AsyncOperation sceneOp = SceneManager.LoadSceneAsync(sceneIndex, LoadSceneMode.Additive);
                    // Set this to true unless we have an activation token
                    sceneOp.allowSceneActivation = (activationToken != null) ? activationToken.AllowSceneActivation : true;
                    loadSceneOps.Add(sceneOp);
                }

                // Now wait for all async operations to complete
                bool completedAllSceneOps = false;

                while (!completedAllSceneOps)
                {
                    if (!Application.isPlaying)
                    {   // Break out of this loop if we've stopped playmode
                        return;
                    }

                    completedAllSceneOps = true;
                    bool readyToProceed = false;
                    bool allowSceneActivation = (activationToken != null) ? activationToken.AllowSceneActivation : true;

                    // Go through all the load scene ops and see if we're ready to be activated
                    float sceneOpProgress = 0;
                    for (int i = 0; i < loadSceneOps.Count; i++)
                    {
                        // Set allow scene activation
                        // (This can be set to true by user before ReadyToProceed is set)
                        loadSceneOps[i].allowSceneActivation = allowSceneActivation;

                        if (loadSceneOps[i].isDone)
                        {   // Sometimes if a scene is small enough, progress will get reset to 0 before you even have a chance to check it
                            // This is true EVEN IF you've set allowSceneActivation to false
                            // So use isDone as a failsafe
                            sceneOpProgress += 1;
                        }
                        else
                        {
                            readyToProceed |= loadSceneOps[i].progress >= SceneActivationLoadProgress;
                            sceneOpProgress += loadSceneOps[i].progress;
                            completedAllSceneOps = false;
                        }
                    }

                    // Let the activation know whether we're ready
                    activationToken?.SetReadyToProceed(readyToProceed);

                    sceneOpProgress = Mathf.Clamp01(SceneOperationProgress / totalSceneOps);
                    
                    SetSceneOpProgress(true, Mathf.Lerp(progressOffset, progressTarget, sceneOpProgress), sceneType);

                    await Task.Yield();
                }
            }

            // Wait for all scenes to be fully loaded before proceeding
            bool scenesLoadedAndActivated = false;
            while (!scenesLoadedAndActivated)
            {
                if (!Application.isPlaying)
                {   // Break out of this loop if we've stopped playmode
                    return;
                }

                scenesLoadedAndActivated = true;
                foreach (int sceneIndex in validIndexes)
                {
                    Scene scene = SceneManager.GetSceneByBuildIndex(sceneIndex);
                    scenesLoadedAndActivated &= (scene.IsValid() & scene.isLoaded);
                }
                await Task.Yield();
            }

            // Make sure our content tracker is refreshed
            contentTracker.RefreshLoadedContent();

            // We're done!
            SetSceneOpProgress(sceneOpInProgressWhenFinished, progressTarget, sceneType);
            
            InvokeLoadedActions(validNames, sceneType);
        }

        /// <summary>
        /// Internal method to handles scene unloads
        /// </summary>
        /// <param name="scenesToUnload"></param>
        /// <param name="sceneType"></param>
        /// <param name="progressOffset"></param>
        /// <param name="progressTarget"></param>
        /// <param name="sceneOpInProgressWhenFinished"></param>
        /// <returns></returns>
        private async Task UnloadScenesInternal(
            IEnumerable<string> scenesToUnload, 
            SceneType sceneType,
            float progressOffset = 0,
            float progressTarget = 1,
            bool sceneOpInProgressWhenFinished = false)
        {
            SetSceneOpProgress(true, progressOffset, sceneType);

            List<string> validNames = new List<string>();
            List<int> validIndexes = new List<int>();

            foreach (string sceneName in scenesToUnload)
            {
                // See if scene exists
                Scene scene;
                int sceneIndex;
                if (!RuntimeSceneUtils.FindScene(sceneName, out scene, out sceneIndex))
                {
                    Debug.LogError("Can't unload invalid scene " + sceneName);
                }
                else
                {
                    validIndexes.Add(sceneIndex);
                    validNames.Add(sceneName);
                }
            }

            int totalSceneOps = validIndexes.Count;
            if (totalSceneOps < 1)
            {
                Debug.LogWarning("No valid scenes found to unload.");
                SetSceneOpProgress(sceneOpInProgressWhenFinished, progressTarget, sceneType);
                return;
            }

            // Invoke our actions
            InvokeWillUnloadActions(validNames, sceneType);

            // Unload our scenes
            if (validIndexes.Count > 0)
            {
                List<AsyncOperation> unloadSceneOps = new List<AsyncOperation>();
                foreach (int sceneIndex in validIndexes)
                {
                    Scene scene = SceneManager.GetSceneByBuildIndex(sceneIndex);
                    if (!scene.isLoaded)
                        continue;

                    AsyncOperation sceneOp = SceneManager.UnloadSceneAsync(sceneIndex);
                    unloadSceneOps.Add(sceneOp);
                }

                // Now wait for all async operations to complete
                bool completedAllSceneOps = false;
                float sceneOpProgress = 0;
                while (!completedAllSceneOps)
                {
                    if (!Application.isPlaying)
                    {   // Break out of this loop if we've stopped playmode
                        return;
                    }

                    completedAllSceneOps = true;
                    sceneOpProgress = 0;
                    for (int i = 0; i < unloadSceneOps.Count; i++)
                    {
                        sceneOpProgress += unloadSceneOps[i].progress;
                        completedAllSceneOps &= unloadSceneOps[i].isDone;
                    }
                    sceneOpProgress = Mathf.Clamp01(SceneOperationProgress / totalSceneOps);

                    SetSceneOpProgress(true, Mathf.Lerp(progressOffset, progressTarget, sceneOpProgress), sceneType);

                    await Task.Yield();
                }
            }

            // Wait for all scenes to be fully unloaded before proceeding
            bool scenesUnloaded = false;
            while (!scenesUnloaded)
            {
                if (!Application.isPlaying)
                {   // Break out of this loop if we've stopped playmode
                    return;
                }

                scenesUnloaded = true;
                foreach (int sceneIndex in validIndexes)
                {
                    Scene scene = SceneManager.GetSceneByBuildIndex(sceneIndex);
                    scenesUnloaded &= !scene.isLoaded;
                }
                await Task.Yield();
            }

            // Make sure our content tracker is refreshed
            contentTracker.RefreshLoadedContent();

            // We're done!
            SetSceneOpProgress(sceneOpInProgressWhenFinished, progressTarget, sceneType);

            // Invoke our actions
            InvokeUnloadedActions(validNames, sceneType);          
        }
        
        private void SetSceneOpProgress(bool inProgress, float progress, SceneType sceneType)
        {
            switch (sceneType)
            {
                case SceneType.Manager:
                    managerSceneOpInProgress = inProgress;
                    managerSceneOpProgress = progress;
                    break;

                case SceneType.Content:
                    SceneOperationInProgress = inProgress;
                    SceneOperationProgress = progress;
                    break;

                case SceneType.Lighting:
                    LightingOperationInProgress = inProgress;
                    LightingOperationProgress = progress;
                    break;

                default:
                    throw new NotImplementedException();
            }
        }

        private bool CanSceneOpProceed(SceneType sceneType)
        {
            switch (sceneType)
            {
                case SceneType.Manager:
                    return !managerSceneOpInProgress;

                case SceneType.Content:
                    return !SceneOperationInProgress;

                case SceneType.Lighting:
                    return !LightingOperationInProgress;

                default:
                    throw new NotImplementedException();
            }
        }

        private void InvokeLoadedActions(List<string> sceneNames, SceneType sceneType)
        {
            foreach (string sceneName in sceneNames)
            {  // Announce scenes individually regardless of type
                OnSceneLoaded?.Invoke(sceneName);
            }

            switch (sceneType)
            {
                case SceneType.Content:
                    // Announce content as a set
                    OnContentLoaded?.Invoke(sceneNames);
                    break;

                case SceneType.Lighting:
                    // We only handle lighting scenes one at a time
                    Debug.Assert(sceneNames.Count == 1);
                    OnLightingLoaded?.Invoke(sceneNames[0]);
                    break;

                default:
                    // Don't announce other types of scenes invidually
                    break;
            }
        }

        private void InvokeWillLoadActions(List<string> sceneNames, SceneType sceneType)
        {
            foreach (string sceneName in sceneNames)
            {   // Announce scenes individually regardless of type
                OnWillLoadScene?.Invoke(sceneName);
            }

            switch (sceneType)
            {
                case SceneType.Content:
                    // Announce content as a set
                    OnWillLoadContent?.Invoke(sceneNames);
                    break;

                case SceneType.Lighting:
                    // We only handle lighting scenes one at a time
                    Debug.Assert(sceneNames.Count == 1);
                    OnWillLoadLighting?.Invoke(sceneNames[0]);
                    break;

                default:
                    // Don't announce other types of scenes invidually
                    break;
            }
        }

        private void InvokeWillUnloadActions(List<string> sceneNames, SceneType sceneType)
        {
            foreach (string sceneName in sceneNames)
            {  // Announce scenes individually regardless of type
                OnWillUnloadScene?.Invoke(sceneName);
            }

            switch (sceneType)
            {
                case SceneType.Content:
                    // Announce content as a set
                    OnWillUnloadContent?.Invoke(sceneNames);
                    break;

                case SceneType.Lighting:
                    // We only handle lighting scenes one at a time
                    Debug.Assert(sceneNames.Count == 1);
                    OnWillUnloadLighting?.Invoke(sceneNames[0]);
                    break;

                default:
                    // Don't announce other types of scenes invidually
                    break;
            }
        }

        private void InvokeUnloadedActions(List<string> sceneNames, SceneType sceneType)
        {
            foreach (string sceneName in sceneNames)
            {  // Announce scenes individually regardless of type
                OnSceneUnloaded?.Invoke(sceneName);
            }

            switch (sceneType)
            {
                case SceneType.Content:
                    // Announce content as a set
                    OnContentUnloaded?.Invoke(sceneNames);
                    break;

                case SceneType.Lighting:
                    // We only handle lighting scenes one at a time
                    Debug.Assert(sceneNames.Count == 1);
                    OnLightingUnloaded?.Invoke(sceneNames[0]);
                    break;

                default:
                    // Don't announce other types of scenes invidually
                    break;
            }
        }
        
        #endregion

        #region Utilities

        /// <inheritdoc />
        public IEnumerable<Scene> GetScenes(IEnumerable<string> sceneNames)
        {
            foreach (string sceneName in sceneNames)
            {
                yield return GetScene(sceneName);
            }
        }

        /// <inheritdoc />
        public Scene GetScene(string sceneName)
        {
            Scene scene = default(Scene);
            RuntimeSceneUtils.FindScene(sceneName, out scene, out int sceneIndex);
            return scene;
        }

        /// <summary>
        /// Checks whether any content scenes are loaded
        /// If they are, adds them to loadedContentScenes and returns true
        /// </summary>
        /// <param name="loadedContentScenes"></param>
        /// <returns></returns>
        private bool GetLoadedContentScenes(out IEnumerable<string> loadedContentScenes)
        {
            List<string> loadedContentScenesList = new List<string>();
            foreach (string sceneName in ContentSceneNames)
            {
                if (IsContentLoaded(sceneName))
                {
                    loadedContentScenesList.Add(sceneName);
                }
            }
            loadedContentScenes = loadedContentScenesList;
            return loadedContentScenesList.Count > 0;
        }

        #endregion

        #region IEqualityComparer

        /// <inheritdoc />
        bool IEqualityComparer.Equals(object x, object y)
        {
            // There shouldn't be other Boundary Managers to compare to.
            return false;
        }

        /// <inheritdoc />
        int IEqualityComparer.GetHashCode(object obj)
        {
            return Mathf.Abs(SourceName.GetHashCode());
        }

        #endregion

        #region Utility Classes
        
        /// <summary>
        /// A utility class used to track which content scenes are loaded, and which should come next / before.
        /// This logic could live in the service itself, but there may be cases where devs want to change how content is tracked without changing anything else.
        /// Might be worth putting this into a SystemType field in the profile.
        /// </summary>
        private sealed class SceneContentTracker
        {
            public SceneContentTracker (MixedRealitySceneSystemProfile profile)
            {
                this.profile = profile;

                CacheSortedContent();               
            }

            private MixedRealitySceneSystemProfile profile;
            public string[] ContentSceneNames => contentSceneNames;
            public SceneInfo[] SortedContentScenes => sortedContentScenes;
            public SceneInfo[] SortedLightingScenes => sortedLightingScenes;

            public bool PrevContentExists { get { return smalledLoadedContentIndex > 0; } }

            public bool NextContentExists { get { return largestLoadedContentIndex < contentSceneNames.Length - 1; } }

            private int largestLoadedContentIndex;
            private int smalledLoadedContentIndex;
            // Cached scene info and scene names
            private string[] contentSceneNames;
            private SceneInfo[] sortedContentScenes;
            private SceneInfo[] sortedLightingScenes;

            private void CacheSortedContent()
            {
                // Store a set of scenes ordered by build index
                sortedContentScenes = profile.ContentScenes.OrderBy(s => s.BuildIndex).ToArray();
                sortedLightingScenes = profile.LightingScenes.OrderBy(s => s.BuildIndex).ToArray();

                // Cache an array of scene names in the same order
                contentSceneNames = new string[sortedContentScenes.Length];
                for (int i = 0; i < contentSceneNames.Length; i++)
                {
                    contentSceneNames[i] = sortedContentScenes[i].Name;
                }
            }

            public bool GetNextContent(bool wrap, out string contentSceneName)
            {
                contentSceneName = string.Empty;
                int nextIndex = largestLoadedContentIndex + 1;
                if (nextIndex >= contentSceneNames.Length)
                {
                    if (wrap)
                    {
                        // If we're wrapping and we've reached the end,
                        // just return the first index.
                        contentSceneName = contentSceneNames[0];
                        return true;
                    }
                    else
                    {   // We're out of scenes!
                        return false;
                    }
                }

                contentSceneName = contentSceneNames[nextIndex];
                return true;
            }

            public bool GetPrevContent(bool wrap, out string contentSceneName)
            {
                contentSceneName = string.Empty;
                int prevIndex = smalledLoadedContentIndex - 1;
                if (prevIndex < 0)
                {
                    if (wrap)
                    {
                        // If we're wrapping and we've reached the start,
                        // just return the last index
                        contentSceneName = contentSceneNames[contentSceneNames.Length - 1];
                        return true;
                    }
                    else
                    {   // We're out of scenes!
                        return false;
                    }
                }

                contentSceneName = contentSceneNames[prevIndex];
                return true;
            }

            public void RefreshLoadedContent()
            {
                largestLoadedContentIndex = -1;
                smalledLoadedContentIndex = contentSceneNames.Length;
                for (int i = 0; i < contentSceneNames.Length; i++)
                {
                    Scene scene = SceneManager.GetSceneByName(contentSceneNames[i]);
                    if (scene.isLoaded)
                    {
                        largestLoadedContentIndex = Mathf.Max(i, largestLoadedContentIndex);
                        smalledLoadedContentIndex = Mathf.Min(i, smalledLoadedContentIndex);
                    }
                }

#if UNITY_EDITOR
                CacheSortedContent();
#endif
            }
        }

        /// <summary>
        /// A utility class used to lerp between and apply lighting settings to the active scene.
        /// </summary>
        private sealed class SceneLightingExecutor
        {
            public void StartTransition(
                RuntimeLightingSettings targetLightingSettings, 
                RuntimeRenderSettings targetRenderSettings,
                RuntimeSunlightSettings targetSunlightSettings,
                LightingSceneTransitionType transitionType = LightingSceneTransitionType.None,
                float transitionDuration = 1)
            {
                // Update our target settings
                this.transitionElapsed = 0;
                this.transitionType = transitionType;
                this.transitionDuration = transitionDuration;
                this.targetLightingSettings = targetLightingSettings;
                this.targetRenderSettings = targetRenderSettings;
                this.targetSunlightSettings = targetSunlightSettings;

                switch (transitionType)
                {
                    case LightingSceneTransitionType.None:
                        // Just execute the transition right now
                        // Zap immediately to the new values
                        currentLightingSettings = targetLightingSettings;
                        currentRenderSettings = targetRenderSettings;
                        currentSunlightSettings = targetSunlightSettings;
                        transitionElapsed = transitionDuration;
                        ApplySettings();
                        return;
                }

                // Otherwise, copy our old settings so we have something to lerp from
                prevLightingSettings = currentLightingSettings;
                prevRenderSettings = currentRenderSettings;
                prevSunlightSettings = currentSunlightSettings;
            }

            public void UpdateTransition(float deltaTime)
            {
                if (transitionElapsed < transitionDuration)
                {
                    transitionElapsed += deltaTime;
                    if (transitionElapsed >= transitionDuration)
                    {
                        currentLightingSettings = targetLightingSettings;
                        currentRenderSettings = targetRenderSettings;
                        currentSunlightSettings = targetSunlightSettings;
                        ApplySettings();
                        return;
                    }
                }

                float transitionProgress = Mathf.Clamp01(transitionElapsed / transitionDuration);

                switch (transitionType)
                {
                    case LightingSceneTransitionType.None:
                        break;

                    case LightingSceneTransitionType.CrossFade:
                        // Just do a straightforward lerp from one setting to the other
                        currentLightingSettings = RuntimeLightingSettings.Lerp(prevLightingSettings, targetLightingSettings, transitionProgress);
                        currentRenderSettings = RuntimeRenderSettings.Lerp(prevRenderSettings, targetRenderSettings, transitionProgress);
                        currentSunlightSettings = RuntimeSunlightSettings.Lerp(prevSunlightSettings, targetSunlightSettings, transitionProgress);
                        break;

                    case LightingSceneTransitionType.FadeToBlack: 
                        // If we're in the first half of our transition, fade out to black
                        if (transitionProgress < 0.5f)
                        {
                            float fadeOutProgress = transitionProgress / 0.5f;
                            currentLightingSettings = RuntimeLightingSettings.Lerp(
                                prevLightingSettings,
                                RuntimeLightingSettings.Black(prevLightingSettings),
                                fadeOutProgress);

                            currentRenderSettings = RuntimeRenderSettings.Lerp(
                                prevRenderSettings,
                                RuntimeRenderSettings.Black(prevRenderSettings),
                                fadeOutProgress);

                            currentSunlightSettings = RuntimeSunlightSettings.Lerp(
                                prevSunlightSettings,
                                RuntimeSunlightSettings.Black(prevSunlightSettings),
                                fadeOutProgress);
                        }
                        else
                        {
                            // If we're in the second half, fade in from black
                            float fadeInProgress = (transitionProgress - 0.5f) / 0.5f;
                            currentLightingSettings = RuntimeLightingSettings.Lerp(
                                RuntimeLightingSettings.Black(targetLightingSettings),
                                targetLightingSettings,
                                fadeInProgress);

                            currentRenderSettings = RuntimeRenderSettings.Lerp(
                                RuntimeRenderSettings.Black(targetRenderSettings),
                                targetRenderSettings,
                                fadeInProgress);

                            currentSunlightSettings = RuntimeSunlightSettings.Lerp(
                                RuntimeSunlightSettings.Black(targetSunlightSettings),
                                targetSunlightSettings,
                                fadeInProgress);
                        }
                        break;
                }

                ApplySettings();
            }

            public void ApplySettings()
            {
                RenderSettings.ambientEquatorColor                  = currentRenderSettings.AmbientEquatorColor;
                RenderSettings.ambientGroundColor                   = currentRenderSettings.AmbientGroundColor;
                RenderSettings.ambientIntensity                     = currentRenderSettings.AmbientIntensity;
                RenderSettings.ambientLight                         = currentRenderSettings.AmbientLight;
                RenderSettings.ambientMode                          = (AmbientMode)currentRenderSettings.AmbientMode;
                RenderSettings.ambientSkyColor                      = currentRenderSettings.AmbientSkyColor;
                RenderSettings.customReflection                     = currentRenderSettings.CustomReflection;
                RenderSettings.defaultReflectionMode                = (DefaultReflectionMode)currentRenderSettings.DefaultReflectionMode;
                RenderSettings.defaultReflectionResolution          = currentRenderSettings.DefaultReflectionResolution;
                RenderSettings.fog                                  = currentRenderSettings.Fog;
                RenderSettings.fogColor                             = currentRenderSettings.FogColor;
                RenderSettings.fogDensity                           = currentRenderSettings.FogDensity;
                RenderSettings.fogEndDistance                       = currentRenderSettings.LinearFogEnd;
                RenderSettings.fogMode                              = currentRenderSettings.FogMode;
                RenderSettings.fogStartDistance                     = currentRenderSettings.LinearFogStart;
                RenderSettings.reflectionBounces                    = currentRenderSettings.ReflectionBounces;
                RenderSettings.reflectionIntensity                  = currentRenderSettings.ReflectionIntensity;
                RenderSettings.skybox                               = currentRenderSettings.SkyboxMaterial;
                RenderSettings.subtractiveShadowColor               = currentRenderSettings.SubtractiveShadowColor;

                if (currentSunlightSettings.UseSunlight)
                {
                    FindOrCreateSunlight();

                    Light sunLight = RenderSettings.sun;
                    sunLight.color = currentSunlightSettings.Color;
                    sunLight.intensity = currentSunlightSettings.Intensity;
                    sunLight.transform.rotation = Quaternion.Euler(currentSunlightSettings.XRotation, currentSunlightSettings.YRotation, currentSunlightSettings.ZRotation);
                }
                else
                {
                    DisableSunlight();
                }
            }

            private void FindOrCreateSunlight()
            {
                if (RenderSettings.sun == null)
                {
                    if (sharedSunLight == null)
                    {
                        Debug.Log("Shared sunlight is null, creating a shared sunlight");
                        // Create a shared sunlight
                        sharedSunLight = new GameObject("Shared Sunlight").AddComponent<Light>();
                        sharedSunLight.type = LightType.Directional;
                        sharedSunLight.intensity = 0;
                    }

                    RenderSettings.sun = sharedSunLight;
                }

                RenderSettings.sun.enabled = true;
            }

            private void DisableSunlight()
            {
                if (RenderSettings.sun != null)
                {
                    RenderSettings.sun.enabled = false;
                }

                if (sharedSunLight != null)
                {
                    sharedSunLight.enabled = false;
                }
            }

            private RuntimeLightingSettings targetLightingSettings;
            private RuntimeLightingSettings currentLightingSettings;
            private RuntimeLightingSettings prevLightingSettings;

            private RuntimeSunlightSettings targetSunlightSettings;
            private RuntimeSunlightSettings currentSunlightSettings;
            private RuntimeSunlightSettings prevSunlightSettings;

            private RuntimeRenderSettings targetRenderSettings;
            private RuntimeRenderSettings currentRenderSettings;
            private RuntimeRenderSettings prevRenderSettings;

            private LightingSceneTransitionType transitionType;
            private float transitionDuration;
            private float transitionElapsed;
            private Light sharedSunLight;
        }

        #endregion
    }
}