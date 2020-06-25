classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    properties (Constant)
        octomap_pa_msgs_AddCloud = 'octomap_pa_msgs/AddCloud'
        octomap_pa_msgs_AddCloudRequest = 'octomap_pa_msgs/AddCloudRequest'
        octomap_pa_msgs_AddCloudResponse = 'octomap_pa_msgs/AddCloudResponse'
        octomap_pa_msgs_AddCloudTf = 'octomap_pa_msgs/AddCloudTf'
        octomap_pa_msgs_AddCloudTfRequest = 'octomap_pa_msgs/AddCloudTfRequest'
        octomap_pa_msgs_AddCloudTfResponse = 'octomap_pa_msgs/AddCloudTfResponse'
        octomap_pa_msgs_Config = 'octomap_pa_msgs/Config'
        octomap_pa_msgs_ConfigBase = 'octomap_pa_msgs/ConfigBase'
        octomap_pa_msgs_ConfigDegrading = 'octomap_pa_msgs/ConfigDegrading'
        octomap_pa_msgs_ConfigInsertion = 'octomap_pa_msgs/ConfigInsertion'
        octomap_pa_msgs_FileName = 'octomap_pa_msgs/FileName'
        octomap_pa_msgs_FileNameRequest = 'octomap_pa_msgs/FileNameRequest'
        octomap_pa_msgs_FileNameResponse = 'octomap_pa_msgs/FileNameResponse'
        octomap_pa_msgs_GetCloud = 'octomap_pa_msgs/GetCloud'
        octomap_pa_msgs_GetCloudRequest = 'octomap_pa_msgs/GetCloudRequest'
        octomap_pa_msgs_GetCloudResponse = 'octomap_pa_msgs/GetCloudResponse'
        octomap_pa_msgs_GetConfig = 'octomap_pa_msgs/GetConfig'
        octomap_pa_msgs_GetConfigRequest = 'octomap_pa_msgs/GetConfigRequest'
        octomap_pa_msgs_GetConfigResponse = 'octomap_pa_msgs/GetConfigResponse'
        octomap_pa_msgs_GetSize = 'octomap_pa_msgs/GetSize'
        octomap_pa_msgs_GetSizeRequest = 'octomap_pa_msgs/GetSizeRequest'
        octomap_pa_msgs_GetSizeResponse = 'octomap_pa_msgs/GetSizeResponse'
        octomap_pa_msgs_Reset = 'octomap_pa_msgs/Reset'
        octomap_pa_msgs_ResetRequest = 'octomap_pa_msgs/ResetRequest'
        octomap_pa_msgs_ResetResponse = 'octomap_pa_msgs/ResetResponse'
        octomap_pa_msgs_SetConfigDegrading = 'octomap_pa_msgs/SetConfigDegrading'
        octomap_pa_msgs_SetConfigDegradingRequest = 'octomap_pa_msgs/SetConfigDegradingRequest'
        octomap_pa_msgs_SetConfigDegradingResponse = 'octomap_pa_msgs/SetConfigDegradingResponse'
        octomap_pa_msgs_SetConfigInsertion = 'octomap_pa_msgs/SetConfigInsertion'
        octomap_pa_msgs_SetConfigInsertionRequest = 'octomap_pa_msgs/SetConfigInsertionRequest'
        octomap_pa_msgs_SetConfigInsertionResponse = 'octomap_pa_msgs/SetConfigInsertionResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(22, 1);
                msgList{1} = 'octomap_pa_msgs/AddCloudRequest';
                msgList{2} = 'octomap_pa_msgs/AddCloudResponse';
                msgList{3} = 'octomap_pa_msgs/AddCloudTfRequest';
                msgList{4} = 'octomap_pa_msgs/AddCloudTfResponse';
                msgList{5} = 'octomap_pa_msgs/Config';
                msgList{6} = 'octomap_pa_msgs/ConfigBase';
                msgList{7} = 'octomap_pa_msgs/ConfigDegrading';
                msgList{8} = 'octomap_pa_msgs/ConfigInsertion';
                msgList{9} = 'octomap_pa_msgs/FileNameRequest';
                msgList{10} = 'octomap_pa_msgs/FileNameResponse';
                msgList{11} = 'octomap_pa_msgs/GetCloudRequest';
                msgList{12} = 'octomap_pa_msgs/GetCloudResponse';
                msgList{13} = 'octomap_pa_msgs/GetConfigRequest';
                msgList{14} = 'octomap_pa_msgs/GetConfigResponse';
                msgList{15} = 'octomap_pa_msgs/GetSizeRequest';
                msgList{16} = 'octomap_pa_msgs/GetSizeResponse';
                msgList{17} = 'octomap_pa_msgs/ResetRequest';
                msgList{18} = 'octomap_pa_msgs/ResetResponse';
                msgList{19} = 'octomap_pa_msgs/SetConfigDegradingRequest';
                msgList{20} = 'octomap_pa_msgs/SetConfigDegradingResponse';
                msgList{21} = 'octomap_pa_msgs/SetConfigInsertionRequest';
                msgList{22} = 'octomap_pa_msgs/SetConfigInsertionResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(9, 1);
                svcList{1} = 'octomap_pa_msgs/AddCloud';
                svcList{2} = 'octomap_pa_msgs/AddCloudTf';
                svcList{3} = 'octomap_pa_msgs/FileName';
                svcList{4} = 'octomap_pa_msgs/GetCloud';
                svcList{5} = 'octomap_pa_msgs/GetConfig';
                svcList{6} = 'octomap_pa_msgs/GetSize';
                svcList{7} = 'octomap_pa_msgs/Reset';
                svcList{8} = 'octomap_pa_msgs/SetConfigDegrading';
                svcList{9} = 'octomap_pa_msgs/SetConfigInsertion';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
