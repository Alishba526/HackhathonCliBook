// Simple Robotics Chatbot for Hackathon
(function() {
    // Create chatbot container
    const chatbotContainer = document.createElement('div');
    chatbotContainer.id = 'robotics-chatbot';
    chatbotContainer.innerHTML = `
        <div id="chatbot-toggle" style="
            position: fixed;
            bottom: 20px;
            right: 20px;
            width: 60px;
            height: 60px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 50%;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            z-index: 1000;
            transition: all 0.3s ease;
        ">
            <span id="chatbot-icon" style="color: white; font-size: 24px;">🤖</span>
        </div>
        <div id="chatbot-window" style="
            position: fixed;
            bottom: 90px;
            right: 20px;
            width: 350px;
            height: 500px;
            background: white;
            border-radius: 12px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
            display: none;
            flex-direction: column;
            z-index: 1000;
            overflow: hidden;
        ">
            <div id="chatbot-header" style="
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                padding: 15px;
                font-weight: bold;
                display: flex;
                justify-content: space-between;
                align-items: center;
            ">
                <span>🤖 Robotics Assistant</span>
                <span id="chatbot-close" style="cursor: pointer; font-size: 20px;">×</span>
            </div>
            <div id="chatbot-messages" style="
                flex: 1;
                padding: 15px;
                overflow-y: auto;
                background: #f8f9fa;
            ">
                <div class="message bot" style="
                    margin-bottom: 10px;
                    padding: 10px;
                    background: white;
                    border-radius: 8px;
                    border-left: 4px solid #667eea;
                ">
                    Hi! I'm your Robotics Assistant. Ask me about ROS 2, Gazebo, Unity, NVIDIA Isaac, or VLA systems! 🤖
                </div>
            </div>
            <div id="chatbot-input" style="
                padding: 15px;
                border-top: 1px solid #e9ecef;
                display: flex;
                gap: 10px;
            ">
                <input type="text" id="chatbot-text" placeholder="Ask me anything..." style="
                    flex: 1;
                    padding: 8px 12px;
                    border: 1px solid #ced4da;
                    border-radius: 6px;
                    outline: none;
                ">
                <button id="chatbot-send" style="
                    padding: 8px 15px;
                    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                    color: white;
                    border: none;
                    border-radius: 6px;
                    cursor: pointer;
                ">Send</button>
            </div>
        </div>
    `;
    document.body.appendChild(chatbotContainer);

    // Chatbot responses
    const responses = {
        'ros': 'ROS 2 is the robotic nervous system! It manages communication between robot components using nodes, topics, and services. Check Module 1 for details.',
        'gazebo': 'Gazebo is a physics simulator for testing robots before real hardware. It uses SDF files and integrates with ROS 2. See Module 2!',
        'unity': 'Unity provides high-fidelity 3D rendering and human-robot interaction simulation. It connects to ROS 2 via TCP. Module 2 covers this.',
        'isaac': 'NVIDIA Isaac provides photorealistic simulation and hardware-accelerated perception. It includes Isaac Sim and Isaac ROS. Module 3 details!',
        'vla': 'Vision-Language-Action combines vision (CLIP), language (Whisper/GPT), and action (ROS 2) for voice-controlled robots. Module 4 explains this.',
        'capstone': 'The capstone project builds an autonomous humanoid that listens to voice commands and executes tasks in simulation. It uses all 4 modules!',
        'modules': 'There are 4 modules: 1) ROS 2 (nervous system), 2) Digital Twin (Gazebo/Unity), 3) AI Brain (Isaac), 4) VLA (voice-action).',
        'default': 'I\'m here to help with robotics questions! Ask about ROS 2, Gazebo, Unity, Isaac, or VLA systems.'
    };

    function getResponse(message) {
        const msg = message.toLowerCase();
        if (msg.includes('ros')) return responses.ros;
        if (msg.includes('gazebo')) return responses.gazebo;
        if (msg.includes('unity')) return responses.unity;
        if (msg.includes('isaac')) return responses.isaac;
        if (msg.includes('vla') || msg.includes('vision') || msg.includes('language') || msg.includes('action')) return responses.vla;
        if (msg.includes('capstone') || msg.includes('project')) return responses.capstone;
        if (msg.includes('module')) return responses.modules;
        return responses.default;
    }

    function addMessage(text, isUser = false) {
        const messages = document.getElementById('chatbot-messages');
        const messageDiv = document.createElement('div');
        messageDiv.className = `message ${isUser ? 'user' : 'bot'}`;
        messageDiv.style.cssText = `
            margin-bottom: 10px;
            padding: 10px;
            border-radius: 8px;
            ${isUser ?
                'background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; margin-left: 50px;' :
                'background: white; border-left: 4px solid #667eea;'
            }
        `;
        messageDiv.textContent = text;
        messages.appendChild(messageDiv);
        messages.scrollTop = messages.scrollHeight;
    }

    // Event listeners
    document.getElementById('chatbot-toggle').addEventListener('click', function() {
        const window = document.getElementById('chatbot-window');
        window.style.display = window.style.display === 'none' ? 'flex' : 'none';
    });

    document.getElementById('chatbot-close').addEventListener('click', function() {
        document.getElementById('chatbot-window').style.display = 'none';
    });

    document.getElementById('chatbot-send').addEventListener('click', function() {
        const input = document.getElementById('chatbot-text');
        const message = input.value.trim();
        if (message) {
            addMessage(message, true);
            setTimeout(() => {
                addMessage(getResponse(message));
            }, 500);
            input.value = '';
        }
    });

    document.getElementById('chatbot-text').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            document.getElementById('chatbot-send').click();
        }
    });
})();