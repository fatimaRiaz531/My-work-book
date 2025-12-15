import React from 'react';

const Message = ({ message, isUser }) => {
  const messageClass = isUser
    ? 'message-user bg-blue-500 text-white'
    : 'message-assistant bg-gray-200 text-gray-800';

  return (
    <div className={`message ${messageClass} max-w-[80%] p-3 rounded-lg mb-3`}>
      <div className="message-content">
        {message}
      </div>
    </div>
  );
};

export default Message;